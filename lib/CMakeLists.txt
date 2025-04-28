# Ethernet
add_library(ETHERNET_FILES STATIC)

target_sources(ETHERNET_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet/socket.c
        ${WIZNET_DIR}/Ethernet/wizchip_conf.c
        )

target_include_directories(ETHERNET_FILES INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5100S
        )

target_link_libraries(ETHERNET_FILES PUBLIC
        W5100S_FILES
        )

add_library(W5100S_FILES STATIC)

target_sources(W5100S_FILES PUBLIC
        ${WIZNET_DIR}/Ethernet/W5100S/w5100s.c
        )

target_include_directories(W5100S_FILES INTERFACE
        ${WIZNET_DIR}/Ethernet
        ${WIZNET_DIR}/Ethernet/W5100S
        )

target_link_libraries(W5100S_FILES PRIVATE
        ETHERNET_FILES
        )

# # Loopback
# add_library(LOOPBACK_FILES STATIC)

# target_sources(LOOPBACK_FILES PUBLIC
#         ${WIZNET_DIR}/Application/loopback/loopback.c
#         )

# target_include_directories(LOOPBACK_FILES PUBLIC
#         ${WIZNET_DIR}/Ethernet
#         ${WIZNET_DIR}/Application/loopback
#         )

# # DHCP
# add_library(DHCP_FILES STATIC)

# target_sources(DHCP_FILES PUBLIC
#         ${WIZNET_DIR}/Internet/DHCP/dhcp.c
#         )

# target_include_directories(DHCP_FILES PUBLIC
#         ${WIZNET_DIR}/Ethernet
#         ${WIZNET_DIR}/Internet/DHCP
#         )

# # DNS
# add_library(DNS_FILES STATIC)

# target_sources(DNS_FILES PUBLIC
#         ${WIZNET_DIR}/Internet/DNS/dns.c
#         )

# target_include_directories(DNS_FILES PUBLIC
#         ${WIZNET_DIR}/Ethernet
#         ${WIZNET_DIR}/Internet/DNS
#         )

# # FTP Client
# add_library(FTPCLIENT_FILES STATIC)

# target_sources(FTPCLIENT_FILES PUBLIC
#         ${WIZNET_DIR}/Internet/FTPClient/ftpc.c
#         )

# target_include_directories(FTPCLIENT_FILES PUBLIC
#         ${WIZNET_DIR}/Ethernet
#         ${WIZNET_DIR}/Internet/FTPClient
#         )

# # FTP Server
# add_library(FTPSERVER_FILES STATIC)

# target_sources(FTPSERVER_FILES PUBLIC
#         ${WIZNET_DIR}/Internet/FTPServer/ftpd.c
#         )

# target_include_directories(FTPSERVER_FILES PUBLIC
#         ${WIZNET_DIR}/Ethernet
#         ${WIZNET_DIR}/Internet/FTPServer
#         )

# # HTTP Server
# add_library(HTTPSERVER_FILES STATIC)

# target_sources(HTTPSERVER_FILES PUBLIC
#         ${WIZNET_DIR}/Internet/httpServer/httpParser.c
#         ${WIZNET_DIR}/Internet/httpServer/httpServer.c
#         ${WIZNET_DIR}/Internet/httpServer/httpUtil.c
#         )

# target_include_directories(HTTPSERVER_FILES PUBLIC
#         ${WIZNET_DIR}/Ethernet
#         ${WIZNET_DIR}/Internet/httpServer
#         )


# # SNTP
# add_library(SNTP_FILES STATIC)

# target_sources(SNTP_FILES PUBLIC
#         ${WIZNET_DIR}/Internet/SNTP/sntp.c
#         )

# target_include_directories(SNTP_FILES PUBLIC
#         ${WIZNET_DIR}/Ethernet
#         ${WIZNET_DIR}/Internet/SNTP
#         )