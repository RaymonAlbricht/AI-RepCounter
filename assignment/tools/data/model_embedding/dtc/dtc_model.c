typedef enum
{
    BicepCurl = 0,
    ChestPress = 1,
    LatPulldown = 2,
    Stationary = 3,
}dtc_t;

/*
 * \brief Decision tree classifier
 * 
 * Decision tree classifier based on the following input characteristics:
 *   BLOCK_SIZE: 175
 *   BLOCK_TYPE: SLIDING
 * 
 * \return dtc_t
 *   0  BicepCurl
 *   1  ChestPress
 *   2  LatPulldown
 *   3  Stationary
 */
dtc_t dtc(const float gyro_x_fir_rescale_variance, const float x_out_fir_rescale_variance, const float y_out_fir_rescale_mean)
{
    dtc_t ret;

    if(gyro_x_fir_rescale_variance <= 16.619078f)
    {
        if(y_out_fir_rescale_mean <= 0.609196f)
        {
            if(x_out_fir_rescale_variance <= 0.002862f)
            {
                 ret = Stationary;
            }
            else // x_out_fir_rescale_variance > 0.002862f
            {
                 ret = ChestPress;
            }
        }
        else // y_out_fir_rescale_mean > 0.609196f
        {
             ret = LatPulldown;
        }
    }
    else // gyro_x_fir_rescale_variance > 16.619078f
    {
         ret = BicepCurl;
    }

    return ret;
}
