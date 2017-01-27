#include <stdio.h>
#include "../core/protocol.h"
#include <cassert>

int main(void){
    struct dqn_tr tr;
    tr.crc = 0;
    tr.id = 1;
    tr.num_slots = 3;

    uint8_t crc = get_crc8((char*)(&tr), sizeof(tr));
    assert(crc == 0x29);
    return 0;
}
