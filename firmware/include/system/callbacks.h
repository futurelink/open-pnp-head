#ifndef CALLBACKS_H
#define CALLBACKS_H

#include "motion.h"
#include "parser.h"

typedef struct {
    motion_block_t  *(*get_current_block)();
    void            (*discard_current_block)();
    uint8_t         (*execute_command)(parser_state_t *parser_state);
} Callbacks;

#endif // CALLBACKS_H
