#ifndef APP_LINKS_H
#define APP_LINKS_H

#include "protocol/protocol_defs.h"
#include "uart_connection.h"

/** One physical protocol endpoint and its byte-stream driver. */
typedef struct {
    proto_link_t id;
    UartConnection* uart;
} AppLink;

/** Indexed by proto_link_t, so IDs and array slots are the same value. */
extern AppLink g_app_links[PROTO_LINK_COUNT];

#endif /* APP_LINKS_H */
