#include "smlk_common.h"

int Str2BCD(const std::string &str, uint8_t buffer[], std::size_t sz)
{
    printf("str.length() = %d\r\n",str.length());
    printf("str.c_str()  = %s\r\n",str.c_str());
    printf("sz  = %d\r\n",sz);
    if ( str.length() > 2 * sz ) {
        return -1;
    }

    std::memset(buffer, 0x00, sz);
    auto index = 0;
    while ( index + str.length() < 2 * sz ) {
        index++;
    }

    for ( auto const num : str ) {
        if ( '0' > num || num > '9' ) {
            return -1;
        }
        buffer[index >> 1] |= (num - '0') << ( (0 == index % 2) ? 4 : 0);
        index++;
    }

    return 0;
}
