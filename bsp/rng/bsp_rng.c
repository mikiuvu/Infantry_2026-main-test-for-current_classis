#include "bsp_rng.h"
#include <stdint.h>
#include "rng.h"
#include "main.h"

int GetRandomInt(int min, int max)
{
    uint32_t random;
    if (HAL_OK != HAL_RNG_GenerateRandomNumber(&hrng, &random))
    {
        return HAL_RNG_ReadLastRandomNumber(&hrng) % (max - min + 1) + min;
    }
    return (random % (max - min + 1)) + min;
    
}
