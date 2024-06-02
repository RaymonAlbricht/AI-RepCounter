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
dtc_t dtc(const float y_out_fir_rescale_mean, const float z_out_fir_rescale_min, const float z_out_fir_rescale_max)
{
    dtc_t ret;

    if(y_out_fir_rescale_mean <= 0.754241f)
    {
        if(z_out_fir_rescale_min <= -0.606537f)
        {
             ret = BicepCurl;
        }
        else // z_out_fir_rescale_min > -0.606537f
        {
            if(z_out_fir_rescale_max <= 0.843368f)
            {
                 ret = Stationary;
            }
            else // z_out_fir_rescale_max > 0.843368f
            {
                 ret = ChestPress;
            }
        }
    }
    else // y_out_fir_rescale_mean > 0.754241f
    {
         ret = LatPulldown;
    }

    return ret;
}
