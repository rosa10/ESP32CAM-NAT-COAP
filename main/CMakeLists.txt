idf_component_register(SRCS "nat_coap.c" 
                            "http_server.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES certs/coap_ca.pem certs/coap_client.crt certs/coap_client.key)
