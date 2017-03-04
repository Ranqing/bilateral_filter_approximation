#ifndef QING_STEREO_H
#define QING_STEREO_H

#include "../../../Qing/qing_common.h"
//#include "../../../Qing/qing_box_filter.h"
//#include "../../../Qing/qing_median_filter.h"
//#include "../../../Qing/qing_guided_filter.h"

class qing_stereo
{
public:
    qing_stereo();
    ~qing_stereo();

public:
    int m_w, m_h, m_disp_range, m_total_size, m_image_size ;
    unsigned char * m_bgr_l, * m_bgr_r;
    float * m_mcost_l, * m_mcost_r;
    float * m_filtered_mcost_l, * m_filtered_mcost_r;
    float * m_range_table, * m_spatial_table;
    double m_sigma_range, m_sigma_spatial;

    void set_params(const int disp_range, const float sigma_range, const float sigma_spatial);
    void load_image(const string filename_l, const string filename_r);
    void compute_mcost_vol_l();
    void compute_mcost_vol_r();
    void aggregate_mcost_vol();

    void appximated_bilateral_filter(float * out, float * in);

    float * get_mcost_vol_l() { return m_mcost_l; }
    float * get_mcost_vol_r() { return m_mcost_r; }

};

#endif // QING_STEREO_H
