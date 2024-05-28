typedef enum
{
    Bicepcurl = 0,
    Chestpress = 1,
    Latpulldown = 2,
    Stationary = 3,
}dtc_t;

/*
 * \brief Decision tree classifier
 * 
 * Decision tree classifier based on the following input characteristics:
 *   BLOCK_SIZE: 100
 *   BLOCK_TYPE: BLOCK
 * 
 * \return dtc_t
 *   0  Bicepcurl
 *   1  Chestpress
 *   2  Latpulldown
 *   3  Stationary
 */
dtc_t dtc(const float y_out_fir_rescale_max, const float z_out_fir_rescale_max)
{
    dtc_t ret;

    if(y_out_fir_rescale_max <= 0.014531f)
    {
         ret = Stationary;
    }
    else // y_out_fir_rescale_max > 0.014531f
    {
        if(z_out_fir_rescale_max <= 0.788158f)
        {
            if(y_out_fir_rescale_max <= 0.945533f)
            {
                 ret = Bicepcurl;
            }
            else // y_out_fir_rescale_max > 0.945533f
            {
                 ret = Latpulldown;
            }
        }
        else // z_out_fir_rescale_max > 0.788158f
        {
             ret = Chestpress;
        }
    }

    return ret;
}
