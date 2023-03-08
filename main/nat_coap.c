#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <sys/param.h>
#include <time.h>
#include <sys/time.h>

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

#include "coap3/coap.h"
#define CAMERA_MODEL_ESP_EYE 1
#define COAP_DEFAULT_TIME_SEC 60
#include <esp_system.h>

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

//dari nat

#include <pthread.h>

#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "esp_vfs_fat.h"
#include "nvs.h"


#include "freertos/event_groups.h"

#include "esp_wpa2.h"

#include "lwip/opt.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "cmd_decl.h"
#include <esp_http_server.h>
// #include "protocol_examples_common.h"
#if !IP_NAPT
#error "IP_NAPT must be defined"
#endif
#include "lwip/lwip_napt.h"

#include "router_globals.h"
// #include "esp_camera.h"
// #include "C:/Users/asus/esp/esp-idf-v4.4.2/examples/common_components/protocol_examples_common/include/protocol_examples_common.h"

// On board LED
#define BLINK_GPIO 2
#define DEFAULT_SCAN_LIST_SIZE 10
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

#define DEFAULT_AP_IP "192.168.4.1"
#define DEFAULT_DNS "8.8.8.8"

/* Global vars */
uint16_t connect_count = 0;
bool ap_connect = false;
bool has_static_ip = false;

uint32_t my_ip;
uint32_t my_ap_ip;

struct portmap_table_entry {
  u32_t daddr;
  u16_t mport;
  u16_t dport;
  u8_t proto;
  u8_t valid;
};
struct portmap_table_entry portmap_tab[IP_PORTMAP_MAX];

esp_netif_t* wifiAP;
esp_netif_t* wifiSTA;

httpd_handle_t start_webserver(void);

#include "esp_camera.h"
#define BOARD_WROVER_KIT 1

// ESP32Cam (AiThinker) PIN Map
#ifdef BOARD_ESP32CAM_AITHINKER
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#endif
#ifdef CAMERA_MODEL_ESP_EYE
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25
#endif
#define EXAMPLE_COAP_PSK_KEY CONFIG_EXAMPLE_COAP_PSK_KEY
#define EXAMPLE_COAP_PSK_IDENTITY CONFIG_EXAMPLE_COAP_PSK_IDENTITY

#define EXAMPLE_COAP_LOG_DEFAULT_LEVEL CONFIG_COAP_LOG_DEFAULT_LEVEL

#define COAP_DEFAULT_DEMO_URI CONFIG_EXAMPLE_TARGET_DOMAIN_URI

const static char *TAG = "CoAP_client";
camera_fb_t *image=NULL;
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 10000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_UXGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 15, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .fb_location=CAMERA_FB_IN_PSRAM,
};

static esp_err_t init_camera()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);  // flip it back
    
    // drop down frame size for higher initial frame rate
    //s->set_framesize(s, FRAMESIZE_96X96);
    s->set_framesize(s, FRAMESIZE_HD);

    return ESP_OK;
}

static int resp_wait = 1;

static coap_optlist_t *optlist = NULL;
static int wait_ms;

#ifdef CONFIG_COAP_MBEDTLS_PKI

extern uint8_t ca_pem_start[] asm("_binary_coap_ca_pem_start");
extern uint8_t ca_pem_end[]   asm("_binary_coap_ca_pem_end");
extern uint8_t client_crt_start[] asm("_binary_coap_client_crt_start");
extern uint8_t client_crt_end[]   asm("_binary_coap_client_crt_end");
extern uint8_t client_key_start[] asm("_binary_coap_client_key_start");
extern uint8_t client_key_end[]   asm("_binary_coap_client_key_end");
#endif /* CONFIG_COAP_MBEDTLS_PKI */
uint64_t start=0;
uint64_t end=0;
int64_t tick_send_image=0;
int64_t tick_send_rssi=0;
static int is_mcast=0;
const char *server_uri = COAP_DEFAULT_DEMO_URI;
static unsigned char _token_data[8];
coap_binary_t base_token={0, _token_data};

static coap_response_t
message_handler(coap_session_t *session,
                const coap_pdu_t *sent,
                const coap_pdu_t *received,
                const coap_mid_t mid)
{
    const unsigned char *data = NULL;
    size_t data_len;
    size_t offset;
    size_t total;
    coap_pdu_code_t rcvd_code = coap_pdu_get_code(received);

    if (COAP_RESPONSE_CLASS(rcvd_code) == 2) {
        if (coap_get_data_large(received, &data_len, &data, &offset, &total)) {
            if (data_len != total) {
                //printf("Unexpected partial data received offset %u, length %u\n", offset, data_len);
            }
            //printf("Received:\n%.*s\n", (int)data_len, data);
            resp_wait = 0;
        }
        end=esp_timer_get_time()-start;
        //printf("ini end %lld", end);
        return COAP_RESPONSE_OK;
    }
    //printf("%d.%02d", (rcvd_code >> 5), rcvd_code & 0x1F);
    if (coap_get_data_large(received, &data_len, &data, &offset, &total)) {
        //printf(": ");
        while(data_len--) {
            //printf("%c", isprint(*data) ? *data : '.');
            data++;
        }
    }
    //printf("\n");
    resp_wait = 0;
    return COAP_RESPONSE_OK;
    
}

#ifdef CONFIG_COAP_MBEDTLS_PKI

static int
verify_cn_callback(const char *cn,
                   const uint8_t *asn1_public_cert,
                   size_t asn1_length,
                   coap_session_t *session,
                   unsigned depth,
                   int validated,
                   void *arg
                  )
{
    coap_log(LOG_INFO, "CN '%s' presented by server (%s)\n",
             cn, depth ? "CA" : "Certificate");
    return 1;
}
#endif /* CONFIG_COAP_MBEDTLS_PKI */

static void
    coap_log_handler (coap_log_t level, const char *message)
{
    uint32_t esp_level = ESP_LOG_INFO;
    char *cp = strchr(message, '\n');
    if (cp)
        ESP_LOG_LEVEL(esp_level, TAG, "%.*s", (int)(cp-message), message);
    else
        ESP_LOG_LEVEL(esp_level, TAG, "%s", message);
}

static coap_address_t *
coap_get_address(coap_uri_t *uri)
{
  static coap_address_t dst_addr;
    char *phostname = NULL;
    struct addrinfo hints;
    struct addrinfo *addrres;
    int error;
    char tmpbuf[INET6_ADDRSTRLEN];

    phostname = (char *)calloc(1, uri->host.length + 1);
    if (phostname == NULL) {
        ESP_LOGE(TAG, "calloc failed");
        return NULL;
    }
    memcpy(phostname, uri->host.s, uri->host.length);
    memset ((char *)&hints, 0, sizeof(hints));
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_family = AF_UNSPEC;

    error = getaddrinfo(phostname, NULL, &hints, &addrres);
    if (error != 0) {
        ESP_LOGE(TAG, "DNS lookup failed for destination address %s. error: %d", phostname, error);
        free(phostname);
        return NULL;
    }
    if (addrres == NULL) {
        ESP_LOGE(TAG, "DNS lookup %s did not return any addresses", phostname);
        free(phostname);
        return NULL;
    }
    free(phostname);
    coap_address_init(&dst_addr);

    switch (addrres->ai_family) {
    case AF_INET:
        memcpy(&dst_addr.addr.sin, addrres->ai_addr, sizeof(dst_addr.addr.sin));
        dst_addr.addr.sin.sin_port        = htons(uri->port);
        inet_ntop(AF_INET, &dst_addr.addr.sin.sin_addr, tmpbuf, sizeof(tmpbuf));
        ESP_LOGI(TAG, "DNS 2lookup succeeded. IP=%s", tmpbuf);
        break;
    case AF_INET6:
        memcpy(&dst_addr.addr.sin6, addrres->ai_addr, sizeof(dst_addr.addr.sin6));
        dst_addr.addr.sin6.sin6_port        = htons(uri->port);
        inet_ntop(AF_INET6, &dst_addr.addr.sin6.sin6_addr, tmpbuf, sizeof(tmpbuf));
        ESP_LOGI(TAG, "DNS3 lookup succeeded. IP=%s", tmpbuf);
        break;
    default:
        ESP_LOGE(TAG, "DNS lookup response failed");
        return NULL;
    }
    freeaddrinfo(addrres);
    return &dst_addr;
}

static int
coap_build_optlist(coap_uri_t *uri)
{
#define BUFSIZE 40
    unsigned char _buf[BUFSIZE];
    unsigned char *buf;
    size_t buflen;
    int res;

    optlist = NULL;

    if (uri->scheme == COAP_URI_SCHEME_COAPS && !coap_dtls_is_supported()) {
        ESP_LOGE(TAG, "MbedTLS (D)TLS Client Mode not configured");
        return 0;
    }
    if (uri->scheme == COAP_URI_SCHEME_COAPS_TCP && !coap_tls_is_supported()) {
        ESP_LOGE(TAG, "CoAP server uri->+tcp:// scheme is not supported");
        return 0;
    }

    if (uri->path.length) {
            buflen = BUFSIZE;
        buf = _buf;
        res = coap_split_path(uri->path.s, uri->path.length, buf, &buflen);

        while (res--) {
            coap_insert_optlist(&optlist,
                                coap_new_optlist(COAP_OPTION_URI_PATH,
                                                 coap_opt_length(buf),
                                                 coap_opt_value(buf)));

            buf += coap_opt_size(buf);
        }
    }

    if (uri->query.length) {
        buflen = BUFSIZE;
        buf = _buf;
        res = coap_split_query(uri->query.s, uri->query.length, buf, &buflen);

        while (res--) {
            coap_insert_optlist(&optlist,
                                coap_new_optlist(COAP_OPTION_URI_QUERY,
                                                 coap_opt_length(buf),
                                                 coap_opt_value(buf)));

            buf += coap_opt_size(buf);
        }
    }
    return 1;
}
#ifdef CONFIG_COAP_MBEDTLS_PSK
static coap_session_t *
coap_start_psk_session(coap_context_t *ctx, coap_address_t *dst_addr, coap_uri_t *uri)
{
 static coap_dtls_cpsk_t dtls_psk;
 static char client_sni[256];

    memset(client_sni, 0, sizeof(client_sni));
    memset (&dtls_psk, 0, sizeof(dtls_psk));
    dtls_psk.version = COAP_DTLS_CPSK_SETUP_VERSION;
    dtls_psk.validate_ih_call_back = NULL;
    dtls_psk.ih_call_back_arg = NULL;
    if (uri->host.length)
        memcpy(client_sni, uri->host.s, MIN(uri->host.length, sizeof(client_sni) - 1));
    else
        memcpy(client_sni, "localhost", 9);
    dtls_psk.client_sni = client_sni;
    dtls_psk.psk_info.identity.s = (const uint8_t *)EXAMPLE_COAP_PSK_IDENTITY;
    dtls_psk.psk_info.identity.length = sizeof(EXAMPLE_COAP_PSK_IDENTITY)-1;
    dtls_psk.psk_info.key.s = (const uint8_t *)EXAMPLE_COAP_PSK_KEY;
    dtls_psk.psk_info.key.length = sizeof(EXAMPLE_COAP_PSK_KEY)-1;
    return coap_new_client_session_psk2(ctx, NULL, dst_addr,
                                       uri->scheme == COAP_URI_SCHEME_COAPS ? COAP_PROTO_DTLS : COAP_PROTO_TLS,
                                       &dtls_psk);
}
#endif /* CONFIG_COAP_MBEDTLS_PSK */

#ifdef CONFIG_COAP_MBEDTLS_PKI
static coap_session_t *
coap_start_pki_session(coap_context_t *ctx, coap_address_t *dst_addr, coap_uri_t *uri)
{
    unsigned int ca_pem_bytes = ca_pem_end - ca_pem_start;
    unsigned int client_crt_bytes = client_crt_end - client_crt_start;
    unsigned int client_key_bytes = client_key_end - client_key_start;
 static coap_dtls_pki_t dtls_pki;
 static char client_sni[256];

    memset (&dtls_pki, 0, sizeof(dtls_pki));
    dtls_pki.version = COAP_DTLS_PKI_SETUP_VERSION;
    if (ca_pem_bytes) {
        
        dtls_pki.verify_peer_cert        = 1;
        dtls_pki.check_common_ca         = 1;
        dtls_pki.allow_self_signed       = 1;
        dtls_pki.allow_expired_certs     = 1;
        dtls_pki.cert_chain_validation   = 1;
        dtls_pki.cert_chain_verify_depth = 2;
        dtls_pki.check_cert_revocation   = 1;
        dtls_pki.allow_no_crl            = 1;
        dtls_pki.allow_expired_crl       = 1;
        dtls_pki.allow_bad_md_hash       = 1;
        dtls_pki.allow_short_rsa_length  = 1;
        dtls_pki.validate_cn_call_back   = verify_cn_callback;
        dtls_pki.cn_call_back_arg        = NULL;
        dtls_pki.validate_sni_call_back  = NULL;
        dtls_pki.sni_call_back_arg       = NULL;
        memset(client_sni, 0, sizeof(client_sni));
        if (uri->host.length) {
            memcpy(client_sni, uri->host.s, MIN(uri->host.length, sizeof(client_sni)));
        } else {
            memcpy(client_sni, "localhost", 9);
        }
        dtls_pki.client_sni = client_sni;
    }
    dtls_pki.pki_key.key_type = COAP_PKI_KEY_PEM_BUF;
    dtls_pki.pki_key.key.pem_buf.public_cert = client_crt_start;
    dtls_pki.pki_key.key.pem_buf.public_cert_len = client_crt_bytes;
    dtls_pki.pki_key.key.pem_buf.private_key = client_key_start;
    dtls_pki.pki_key.key.pem_buf.private_key_len = client_key_bytes;
    dtls_pki.pki_key.key.pem_buf.ca_cert = ca_pem_start;
    dtls_pki.pki_key.key.pem_buf.ca_cert_len = ca_pem_bytes;

    return coap_new_client_session_pki(ctx, NULL, dst_addr,
                                              uri->scheme == COAP_URI_SCHEME_COAPS ? COAP_PROTO_DTLS : COAP_PROTO_TLS,
                                              &dtls_pki);
}
#endif /* CONFIG_COAP_MBEDTLS_PKI */

void send_image(coap_session_t *session){ //tambahin *tick
    coap_pdu_t *request=NULL;
    uint8_t token[8];
    size_t tokenlen;     
    coap_optlist_t *optlist=NULL;
    char *image_path="image";
    // wifi_scan();
    coap_pdu_t *requestrssi=NULL;
    uint8_t tokenrssi[8];
    size_t tokenlenrssi;     
    coap_optlist_t *optlistrssi=NULL;
    wifi_ap_record_t ap;
    esp_wifi_sta_get_ap_info(&ap);
    printf("iya ini%d\n", ap.rssi);
    image=esp_camera_fb_get();
    
    //if (esp_timer_get_timer()-*tick>5000000) supaya 5 detik sekali
    //tik=esptimergettimer

    if (!image){
        coap_log(LOG_NOTICE,"Take image failed!\n");
        goto clean_up;
    }
    
    // send_duration = esp_timer_get_time();
    coap_log(LOG_NOTICE, "Start sending image, start tick %lld\n", esp_timer_get_time());
    
    if (!(request = coap_new_pdu(COAP_MESSAGE_CON,COAP_REQUEST_CODE_PUT,session))){ //get
        ESP_LOGE(TAG,"coap_new_pdu() failed");
    }
    coap_session_new_token(session, &tokenlen, token);
    //track_new_token(tokenlen, token);
    if (!coap_add_token(request,tokenlen,token)){
        coap_log(LOG_DEBUG,"cannot add token to request\n");
    }
    
    // if (!(request = coap_new_pdu(COAP_MESSAGE_CON,COAP_REQUEST_CODE_PUT,sessionrssi))){ //get
    //     ESP_LOGE(TAG,"coap_new_pdu() failed");
    // }
    // coap_session_new_token(sessionrssi, &tokenlenrssi, tokenrssi);
    // //track_new_token(tokenlen, token);
    // if (!coap_add_token(requestrssi,tokenlenrssi,tokenrssi)){
    //     coap_log(LOG_DEBUG,"cannot add token to request\n");
    // }
    coap_add_option(request, COAP_OPTION_URI_PATH, 5, (uint8_t *)image_path); //path yg baru
    coap_add_option(request,COAP_OPTION_URI_QUERY,1, (uint8_t *)"1");
    coap_add_data_large_request(session,request,image->len, image->buf, NULL, NULL);
    // coap_add_option(requestrssi,COAP_OPTION_URI_PATH,4,(uint8_t *)"rssi");
    resp_wait=1;
    start= esp_timer_get_time();
    
    //printf("ini start: %lld", start);
    coap_send(session, request);
    // coap_send(sessionrssi,requestrssi);
    
    clean_up:
    
    if (optlist) {
        coap_delete_optlist(optlist);
        optlist = NULL;
    }
    
    
}

static int
resolve_address(const coap_str_const_t *server, struct sockaddr *dst) {

  struct addrinfo *res, *ainfo;
  struct addrinfo hints;
  static char addrstr[256];
  int error, len=-1;

  memset(addrstr, 0, sizeof(addrstr));
  if (server->length)
    memcpy(addrstr, server->s, server->length);
  else
    memcpy(addrstr, "localhost", 9);

  memset ((char *)&hints, 0, sizeof(hints));
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_family = AF_UNSPEC;

  error = getaddrinfo(addrstr, NULL, &hints, &res);

  if (error != 0) {
    //fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(error));
    return error;
  }

  for (ainfo = res; ainfo != NULL; ainfo = ainfo->ai_next) {
    switch (ainfo->ai_family) {
    case AF_INET6:
    case AF_INET:
      len = (int)ainfo->ai_addrlen;
      memcpy(dst, ainfo->ai_addr, len);
      goto finish;
    default:
      ;
    }
  }

 finish:
  freeaddrinfo(res);
  return len;
}

static coap_session_t*
open_session(
  coap_context_t *ctx,
  coap_proto_t proto,
  coap_address_t *bind_addr,
  coap_address_t *dst,
  const uint8_t *identity,
  size_t identity_len,
  const uint8_t *key,
  size_t key_len
) {
  coap_session_t *session;

    /* Non-encrypted session */
    session = coap_new_client_session(ctx, bind_addr, dst, proto);
  
  return session;
}

static coap_session_t *
get_session(
  coap_context_t *ctx,
  const char *local_addr,
  const char *local_port,
  coap_proto_t proto,
  coap_address_t *dst,
  const uint8_t *identity,
  size_t identity_len,
  const uint8_t *key,
  size_t key_len
) {
  coap_session_t *session = NULL;

  is_mcast = coap_is_mcast(dst);
  if ( local_addr ) {
    int s;
    struct addrinfo hints;
    struct addrinfo *result = NULL, *rp;

    memset( &hints, 0, sizeof( struct addrinfo ) );
    hints.ai_family = AF_UNSPEC;    /* Allow IPv4 or IPv6 */
    hints.ai_socktype = COAP_PROTO_RELIABLE(proto) ? SOCK_STREAM : SOCK_DGRAM; /* Coap uses UDP */
    hints.ai_flags = AI_PASSIVE | AI_NUMERICHOST | AI_NUMERICSERV | AI_ALL;

    s = getaddrinfo( local_addr, local_port, &hints, &result );
    if ( s != 0 ) {
     
      return NULL;
    }

    /* iterate through results until success */
    for ( rp = result; rp != NULL; rp = rp->ai_next ) {
      coap_address_t bind_addr;
      if ( rp->ai_addrlen <= (socklen_t)sizeof( bind_addr.addr ) ) {
        coap_address_init( &bind_addr );
        bind_addr.size = (socklen_t)rp->ai_addrlen;
        memcpy( &bind_addr.addr, rp->ai_addr, rp->ai_addrlen );
        session = open_session(ctx, proto, &bind_addr, dst,
                               identity, identity_len, key, key_len);
        if ( session )
          break;
      }
    }
    freeaddrinfo( result );
  } else if (local_port) {
    coap_address_t bind_addr;

    coap_address_init(&bind_addr);
    bind_addr.size = dst->size;
    bind_addr.addr.sa.sa_family = dst->addr.sa.sa_family;
    /* port is in same place for IPv4 and IPv6 */
    bind_addr.addr.sin.sin_port = ntohs(atoi(local_port));
    session = open_session(ctx, proto, &bind_addr, dst,
                               identity, identity_len, key, key_len);
  } else {
    session = open_session(ctx, proto, NULL, dst,
                               identity, identity_len, key, key_len);
  }
  return session;
}

void prepare_session(coap_context_t *ctx, coap_session_t **session, int64_t *tick) {
    coap_address_t dst;
    static coap_uri_t uri;
    static coap_str_const_t server;
    int res;

    if (coap_split_uri((const uint8_t *)server_uri, strlen(server_uri), &uri) == -1)
      {ESP_LOGE(TAG, "CoAP server uri error"); }

    server = uri.host;
    coap_address_init(&dst);
    res = resolve_address(&server, &dst.addr.sa);
    if (res < 0) {
      fprintf(stderr, "failed to resolve address\n");
      exit(-1);
    }
    dst.size = res;
    dst.addr.sin.sin_port = htons(COAP_DEFAULT_PORT);

    *session = get_session(
      ctx,NULL, "0",COAP_PROTO_UDP,
      &dst,NULL,0,NULL,0
    );

    if ( !session ) {
      coap_log( LOG_EMERG, "cannot create client session\n" );
    }
    coap_session_init_token(*session, base_token.length, base_token.s);
    *tick = esp_timer_get_time();
}





void * coap_example_client(void *p)
{
    //ESP_LOGI(TAG,"CoAP client running");
    coap_address_t   *dst_addr;
    static coap_uri_t uri;
    
    coap_context_t *ctx = NULL;
    coap_session_t *session = NULL;
    coap_session_t *session_image = NULL;
    // coap_session_t *session_tp = NULL;
    // coap_session_t *session_delay = NULL;
    //coap_pdu_t *request = NULL;
    //unsigned char token[8];
    //size_t tokenlength;

    /* Set up the CoAP logging */
    coap_startup();
    coap_set_log_handler(coap_log_handler);
    coap_set_log_level(EXAMPLE_COAP_LOG_DEFAULT_LEVEL);
    /* Set up the CoAP context */
    ctx = coap_new_context(NULL);
    if (!ctx) {
        ESP_LOGE(TAG, "coap_new_context() failed");
        goto clean_up;
    }
    coap_context_set_keepalive(ctx,0);
    coap_context_set_block_mode(ctx,
                                COAP_BLOCK_USE_LIBCOAP);

    coap_register_response_handler(ctx, message_handler);
    // prepare_session(ctx,&session,&tick_send_rssi );
    prepare_session(ctx,&session_image, &tick_send_image);
    //session_tp
    // wifi_scan();
    for(;;){
        // printf("ini ngirim");
        send_image(session_image); 
    
    //get prediction, put->get
        wait_ms = COAP_DEFAULT_TIME_SEC * 1000;

        while (resp_wait) {
            int result = coap_io_process(ctx, wait_ms > 1000 ? 1000 : wait_ms);
            if (result >= 0) {
                if (result >= wait_ms) {
                    ESP_LOGE(TAG, "No response from server");
                    break;
                } else {
                    wait_ms -= result;
                }
            }
        }
        //ESP_LOGI(TAG, "Starting again!");
        clean_up:
            esp_camera_fb_return(image);
        }
        }
    

//dari nat
#if CONFIG_STORE_HISTORY

#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"

static void initialize_filesystem(void)
{
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}
#endif // CONFIG_STORE_HISTORY

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

esp_err_t apply_portmap_tab() {
    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (portmap_tab[i].valid) {
            ip_portmap_add(portmap_tab[i].proto, my_ip, portmap_tab[i].mport, portmap_tab[i].daddr, portmap_tab[i].dport);
        }
    }
    return ESP_OK;
}

esp_err_t delete_portmap_tab() {
    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (portmap_tab[i].valid) {
            ip_portmap_remove(portmap_tab[i].proto, portmap_tab[i].mport);
        }
    }
    return ESP_OK;
}

void print_portmap_tab() {
    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (portmap_tab[i].valid) {
            printf ("%s", portmap_tab[i].proto == PROTO_TCP?"TCP ":"UDP ");
            ip4_addr_t addr;
            addr.addr = my_ip;
            printf (IPSTR":%d -> ", IP2STR(&addr), portmap_tab[i].mport);
            addr.addr = portmap_tab[i].daddr;
            printf (IPSTR":%d\n", IP2STR(&addr), portmap_tab[i].dport);
        }
    }
}

esp_err_t get_portmap_tab() {
    esp_err_t err;
    nvs_handle_t nvs;
    size_t len;

    err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_get_blob(nvs, "portmap_tab", NULL, &len);
    if (err == ESP_OK) {
        if (len != sizeof(portmap_tab)) {
            err = ESP_ERR_NVS_INVALID_LENGTH;
        } else {
            err = nvs_get_blob(nvs, "portmap_tab", portmap_tab, &len);
            if (err != ESP_OK) {
                memset(portmap_tab, 0, sizeof(portmap_tab));
            }
        }
    }
    nvs_close(nvs);

    return err;
}
esp_err_t add_portmap(u8_t proto, u16_t mport, u32_t daddr, u16_t dport) {
    esp_err_t err;
    nvs_handle_t nvs;

    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (!portmap_tab[i].valid) {
            portmap_tab[i].proto = proto;
            portmap_tab[i].mport = mport;
            portmap_tab[i].daddr = daddr;
            portmap_tab[i].dport = dport;
            portmap_tab[i].valid = 1;

            err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
            if (err != ESP_OK) {
                return err;
            }
            err = nvs_set_blob(nvs, "portmap_tab", portmap_tab, sizeof(portmap_tab));
            if (err == ESP_OK) {
                err = nvs_commit(nvs);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "New portmap table stored.");
                }
            }
            nvs_close(nvs);

            ip_portmap_add(proto, my_ip, mport, daddr, dport);

            return ESP_OK;
        }
    }
    return ESP_ERR_NO_MEM;
}

esp_err_t del_portmap(u8_t proto, u16_t mport) {
    esp_err_t err;
    nvs_handle_t nvs;

    for (int i = 0; i<IP_PORTMAP_MAX; i++) {
        if (portmap_tab[i].valid && portmap_tab[i].mport == mport && portmap_tab[i].proto == proto) {
            portmap_tab[i].valid = 0;

            err = nvs_open(PARAM_NAMESPACE, NVS_READWRITE, &nvs);
            if (err != ESP_OK) {
                return err;
            }
            err = nvs_set_blob(nvs, "portmap_tab", portmap_tab, sizeof(portmap_tab));
            if (err == ESP_OK) {
                err = nvs_commit(nvs);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "New portmap table stored.");
                }
            }
            nvs_close(nvs);

            ip_portmap_remove(proto, mport);
            return ESP_OK;
        }
    }
    return ESP_OK;
}

static void initialize_console(void)
{
    /* Drain stdout before reconfiguring it */
    fflush(stdout);
    fsync(fileno(stdout));

    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_port_set_rx_line_endings(0, ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_port_set_tx_line_endings(0, ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    const uart_config_t uart_config = {
            .baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            #if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
                .source_clk = UART_SCLK_REF_TICK,
            #else
                .source_clk = UART_SCLK_XTAL,
            #endif
    };
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
            256, 0, 0, NULL, 0) );
    ESP_ERROR_CHECK( uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    /* Initialize the console */
    esp_console_config_t console_config = {
            .max_cmdline_args = 8,
            .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
            .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK( esp_console_init(&console_config) );

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);

#if CONFIG_STORE_HISTORY
    /* Load command history from filesystem */
    linenoiseHistoryLoad(HISTORY_PATH);
#endif
}

void * led_status_thread(void * p)
{
    
    
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (true)
    {
        gpio_set_level(BLINK_GPIO, ap_connect);

        for (int i = 0; i < connect_count; i++)
        {
            gpio_set_level(BLINK_GPIO, 1 - ap_connect);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            gpio_set_level(BLINK_GPIO, ap_connect);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    esp_netif_dns_info_t dns;
    ip_addr_t dnsserver;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG,"disconnected - retry to connect to the AP");
        ap_connect = false;
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        if (esp_netif_get_dns_info(wifiSTA, ESP_NETIF_DNS_MAIN, &dns) == ESP_OK)
        {
            dnsserver.type = IPADDR_TYPE_V4;
            dnsserver.u_addr.ip4.addr = dns.ip.u_addr.ip4.addr;
            dhcps_dns_setserver(&dnsserver);
            ESP_LOGI(TAG, "set dns to:" IPSTR, IP2STR(&(dnsserver.u_addr.ip4)));
        }
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        connect_count++;
        ESP_LOGI(TAG,"%d. station connected", connect_count);
    }
    else if (event_base == IP_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        connect_count--;
        ESP_LOGI(TAG,"station disconnected - %d remain", connect_count);
    }
}

const int CONNECTED_BIT = BIT0;
#define JOIN_TIMEOUT_MS (2000)

void wifi_init(const char* ssid, const char* ent_username, const char* ent_identity, const char* passwd, const char* static_ip, const char* subnet_mask, const char* gateway_addr, const char* ap_ssid, const char* ap_passwd, const char* ap_ip)
{
    ip_addr_t dnsserver;
    //tcpip_adapter_dns_info_t dnsinfo;

    wifi_event_group = xEventGroupCreate();
  
    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifiAP = esp_netif_create_default_wifi_ap();
    wifiSTA = esp_netif_create_default_wifi_sta();

    tcpip_adapter_ip_info_t ipInfo_sta;
    if ((strlen(ssid) > 0) && (strlen(static_ip) > 0) && (strlen(subnet_mask) > 0) && (strlen(gateway_addr) > 0)) {
        has_static_ip = true;
        my_ip = ipInfo_sta.ip.addr = ipaddr_addr(static_ip);
        ipInfo_sta.gw.addr = ipaddr_addr(gateway_addr);
        ipInfo_sta.netmask.addr = ipaddr_addr(subnet_mask);
        tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA); // Don't run a DHCP client
        tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo_sta);
        apply_portmap_tab();
    }

    my_ap_ip = ipaddr_addr(ap_ip);

    esp_netif_ip_info_t ipInfo_ap;
    ipInfo_ap.ip.addr = my_ap_ip;
    ipInfo_ap.gw.addr = my_ap_ip;
    IP4_ADDR(&ipInfo_ap.netmask, 255,255,255,0);
    esp_netif_dhcps_stop(wifiAP); // stop before setting ip WifiAP
    esp_netif_set_ip_info(wifiAP, &ipInfo_ap);
    esp_netif_dhcps_start(wifiAP);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* ESP WIFI CONFIG */
    wifi_config_t wifi_config = { 0 };
        wifi_config_t ap_config = {
        .ap = {
            .channel = 0,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .ssid_hidden = 0,
            .max_connection = 8,
            .beacon_interval = 100,
        }
    };

    strlcpy((char*)ap_config.sta.ssid, ap_ssid, sizeof(ap_config.sta.ssid));
    if (strlen(ap_passwd) < 8) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    } else {
	    strlcpy((char*)ap_config.sta.password, ap_passwd, sizeof(ap_config.sta.password));
    }

    if (strlen(ssid) > 0) {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA) );

        //Set SSID
        strlcpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
        //Set passwprd
        if(strlen(ent_username) == 0) {
            ESP_LOGI(TAG, "STA regular connection");
            strlcpy((char*)wifi_config.sta.password, passwd, sizeof(wifi_config.sta.password));
        }
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
        if(strlen(ent_username) != 0 && strlen(ent_identity) != 0) {
            ESP_LOGI(TAG, "STA enterprise connection");
            if(strlen(ent_username) != 0 && strlen(ent_identity) != 0) {
                esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)ent_identity, strlen(ent_identity)); //provide identity
            } else {
                esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)ent_username, strlen(ent_username));
            }
            esp_wifi_sta_wpa2_ent_set_username((uint8_t *)ent_username, strlen(ent_username)); //provide username
            esp_wifi_sta_wpa2_ent_set_password((uint8_t *)passwd, strlen(passwd)); //provide password
            esp_wifi_sta_wpa2_ent_enable();
        }

        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config) );
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP) );
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config) );        
    }

    // Enable DNS (offer) for dhcp server
    dhcps_offer_t dhcps_dns_value = OFFER_DNS;
    dhcps_set_option_info(6, &dhcps_dns_value, sizeof(dhcps_dns_value));

    // Set custom dns server address for dhcp server
    dnsserver.u_addr.ip4.addr = ipaddr_addr(DEFAULT_DNS);;
    dnsserver.type = IPADDR_TYPE_V4;
    dhcps_dns_setserver(&dnsserver);

//    tcpip_adapter_get_dns_info(TCPIP_ADAPTER_IF_AP, TCPIP_ADAPTER_DNS_MAIN, &dnsinfo);
//    ESP_LOGI(TAG, "DNS IP:" IPSTR, IP2STR(&dnsinfo.ip.u_addr.ip4));

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
        pdFALSE, pdTRUE, JOIN_TIMEOUT_MS / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(esp_wifi_start());

    if (strlen(ssid) > 0) {
        ESP_LOGI(TAG, "wifi_init_apsta finished.");
        ESP_LOGI(TAG, "connect to ap SSID: %s ", ssid);
    } else {
        ESP_LOGI(TAG, "wifi_init_ap with default finished.");      
    }
}

char* ssid = NULL;
char* ent_username = NULL;
char* ent_identity = NULL;
char* passwd = NULL;
char* static_ip = NULL;
char* subnet_mask = NULL;
char* gateway_addr = NULL;
char* ap_ssid = NULL;
char* ap_passwd = NULL;
char* ap_ip = NULL;

char* param_set_default(const char* def_val) {
    char * retval = malloc(strlen(def_val)+1);
    strcpy(retval, def_val);
    return retval;
}

void app_main(void)
{
    
    initialize_nvs();
    if(ESP_OK != init_camera()) {
        return;
    }
    
#if CONFIG_STORE_HISTORY
    initialize_filesystem();
    ESP_LOGI(TAG, "Command history enabled");
#else
    ESP_LOGI(TAG, "Command history disabled");
#endif

    get_config_param_str("ssid", &ssid);
    if (ssid == NULL) {
        ssid = param_set_default("dd-wrt");
    }
    get_config_param_str("ent_username", &ent_username);
    if (ent_username == NULL) {
        ent_username = param_set_default("");
    }
    get_config_param_str("ent_identity", &ent_identity);
    if (ent_identity == NULL) {
        ent_identity = param_set_default("");
    }
    get_config_param_str("passwd", &passwd);
    if (passwd == NULL) {
        passwd = param_set_default("dspmcitb");
    }
    get_config_param_str("static_ip", &static_ip);
    if (static_ip == NULL) {
        static_ip = param_set_default("");
    }
    get_config_param_str("subnet_mask", &subnet_mask);
    if (subnet_mask == NULL) {
        subnet_mask = param_set_default("");
    }
    get_config_param_str("gateway_addr", &gateway_addr);
    if (gateway_addr == NULL) {
        gateway_addr = param_set_default("");
    }
    get_config_param_str("ap_ssid", &ap_ssid);
    if (ap_ssid == NULL) {
        ap_ssid = param_set_default("ESP32_NAT_Router");
    }   
    get_config_param_str("ap_passwd", &ap_passwd);
    if (ap_passwd == NULL) {
        ap_passwd = param_set_default("");
    }
    get_config_param_str("ap_ip", &ap_ip);
    if (ap_ip == NULL) {
        ap_ip = param_set_default(DEFAULT_AP_IP);
    }

    get_portmap_tab();

    // Setup WIFI
    wifi_init(ssid, ent_username, ent_identity, passwd, static_ip, subnet_mask, gateway_addr, ap_ssid, ap_passwd, ap_ip);

    pthread_t t1;
    
    pthread_create(&t1, NULL, led_status_thread, NULL);
    
    ip_napt_enable(my_ap_ip, 1);
    ESP_LOGI(TAG, "NAT is enabled");

    char* lock = NULL;
    get_config_param_str("lock", &lock);
    if (lock == NULL) {
        lock = param_set_default("0");
    }
    if (strcmp(lock, "0") ==0) {
        ESP_LOGI(TAG,"Starting config web server");
        start_webserver();
    }
    free(lock);

    initialize_console();

    /* Register commands */
    esp_console_register_help_command();
    register_system();
    register_nvs();
    register_router();

    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    const char* prompt = LOG_COLOR_I "esp32> " LOG_RESET_COLOR;
    // coap_example_client(NULL);
    // printf("\n"
    //        "ESP32 NAT ROUTER\n"
    //        "Type 'help' to get the list of commands.\n"
    //        "Use UP/DOWN arrows to navigate through command history.\n"
    //        "Press TAB when typing command name to auto-complete.\n");
    
    if (strlen(ssid) == 0) {
        //  printf("\n"
        //        "Unconfigured WiFi\n"
        //        "Configure using 'set_sta' and 'set_ap' and restart.\n");       
    }

    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        // printf("\n"
        //        "Your terminal application does not support escape sequences.\n"
        //        "Line editing and history features are disabled.\n"
        //        "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = "esp32> ";
#endif //CONFIG_LOG_COLORS
    }
coap_example_client(NULL);
    /* Main loop */
//     while(true) {
//         /* Get a line using linenoise.
//          * The line is returned when ENTER is pressed.
//          */
//         char* line = linenoise(prompt);
//         if (line == NULL) { /* Ignore empty lines */
//             continue;
//         }
//         /* Add the command to the history */
//         linenoiseHistoryAdd(line);
// #if CONFIG_STORE_HISTORY
//         /* Save command history to filesystem */
//         linenoiseHistorySave(HISTORY_PATH);
// #endif

//         /* Try to run the command */
//         int ret;
//         esp_err_t err = esp_console_run(line, &ret);
//         if (err == ESP_ERR_NOT_FOUND) {
//             printf("Unrecognized command\n");
//         } else if (err == ESP_ERR_INVALID_ARG) {
//             // command was empty
//         } else if (err == ESP_OK && ret != ESP_OK) {
//             printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
//         } else if (err != ESP_OK) {
//             printf("Internal error: %s\n", esp_err_to_name(err));
//         }
//         /* linenoise allocates line buffer on the heap, so need to free it */
//         linenoiseFree(line);
//     }
    // ESP_ERROR_CHECK( nvs_flash_init() ); 
    // ESP_ERROR_CHECK(esp_netif_init()); //init ip
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
    // ESP_ERROR_CHECK(example_connect()); //connect wifi
    // if(ESP_OK != init_camera()) {
    //     return;
    // }
    // pthread_t t2;
    // pthread_create(&t2, NULL, coap_example_client, NULL);
    
}
       

