#include "qing_stereo.h"

#include "../../../Qing/qing_macros.h"
#include "../../../Qing/qing_string.h"
#include "../../../Qing/qing_matching_cost.h"
#include "../../../Qing/qing_bilateral_filter.h"
#include "../../../Qing/qing_dir.h"
#include "../../../Qing/qing_timer.h"

qing_stereo::qing_stereo(): m_bgr_l(0), m_bgr_r(0), m_mcost_l(0), m_mcost_r(0), m_filtered_mcost_l(0), m_filtered_mcost_r(0)
{
}

qing_stereo::~qing_stereo() {
    if(m_bgr_l!=0) {delete[] m_bgr_l;m_bgr_l=0;}
    if(m_bgr_r!=0) {delete[] m_bgr_r;m_bgr_r=0;}
    if(m_mcost_l!=0) {delete[] m_mcost_l;m_mcost_l=0;}
    if(m_mcost_r!=0) {delete[] m_mcost_r;m_mcost_r=0;}
    if(m_filtered_mcost_l!=0) {delete[] m_filtered_mcost_l; m_filtered_mcost_l = 0; }
    if(m_filtered_mcost_r!=0) {delete[] m_filtered_mcost_r; m_filtered_mcost_r = 0; }

}

void qing_stereo::load_image(const string filename_l, const string filename_r) {
    Mat mat_l = imread(filename_l, 1);
    if(mat_l.data == NULL) {
        cerr << "failed to open " << filename_l << endl;
        return ;
    }
    Mat mat_r = imread(filename_r, 1);
    if(mat_r.data == NULL) {
        cerr << "failed to open " << filename_r << endl;
        return ;
    }

    m_w = mat_l.size().width;
    m_h = mat_l.size().height;
    m_image_size = m_w * m_h;
    m_total_size = m_image_size * m_disp_range;

    m_bgr_l = new unsigned char[m_image_size * 3];
    m_bgr_r = new unsigned char[m_image_size * 3];
    memcpy(m_bgr_l, mat_l.data, sizeof(unsigned char) * m_image_size * 3);
    memcpy(m_bgr_r, mat_r.data, sizeof(unsigned char) * m_image_size * 3);
    m_mcost_l = new float[m_total_size];
    m_mcost_r = new float[m_total_size];
    if(m_mcost_l == 0) {
        cerr << "failed to new left matching cost.. " << endl;
        exit(-1);
    }
    if(m_mcost_r == 0) {
        cerr << "failed to new right matching cost.." << endl;
        exit(-1);
    }
    //    m_filtered_mcost_l = new float[m_total_size];
    //    m_filtered_mcost_r = new float[m_total_size];
    cout << QING_DEBUG_FLAG_STRING << endl;
}

void qing_stereo::set_params(const int disp_range, const float sigma_range, const float sigma_spatial) {
    m_disp_range = disp_range;
    m_sigma_range = sigma_range * QING_FILTER_INTENSITY_RANGE;
    m_sigma_spatial = sigma_spatial * min(m_w, m_h);
    //    cout << "sigma_range = " << m_sigma_range << endl;
    //    cout << "sigma_spatial = " << m_sigma_spatial << endl;

    //    m_range_table = qing_get_range_weighted_table(m_sigma_range, QING_FILTER_INTENSITY_RANGE);
    //    m_spatial_table = qing_get_spatial_weighted_table(m_sigma_spatial, 2*(int)(m_sigma_spatial+0.5f)+1);
    cout << QING_DEBUG_FLAG_STRING << endl;
}

void qing_stereo::compute_mcost_vol_l(){
    memset(m_mcost_l, 0.f, sizeof(float)*m_total_size);
    for(int d = 0; d < m_disp_range; ++d) {
        float * mcost = m_mcost_l + d * m_image_size;
        for(int y = 0, idx = 0; y < m_h; ++y) {
            for(int x = 0; x < m_w; ++x) {
                if(x-d<0) {idx++;continue;}
                mcost[idx] = qing_get_mcost_tad( m_bgr_l+idx*3, m_bgr_r+(idx-d)*3, 3);
                idx ++ ;
            }
        }
    }

# if 0
    qing_create_dir("matching-cost");
    for(int d = 0; d < m_disp_range; ++d) {
        string out_txt_file = "./matching-cost/mcost_l_" + qing_int_2_string(d) + ".txt";
        string out_jpg_file = "./matching-cost/mcost_l_" + qing_int_2_string(d) + ".jpg";
        float * mcost = m_mcost_l + d * m_image_size;
        qing_save_mcost_txt(out_txt_file, mcost, m_image_size, m_w);
        qing_save_mcost_jpg(out_jpg_file, mcost, m_w, m_h);
    }
# endif
}

void qing_stereo::compute_mcost_vol_r(){
    memset(m_mcost_r, 0.f, sizeof(float)*m_total_size);
    for(int d = 0; d < m_disp_range; ++d) {
        float * mcost = m_mcost_r + d * m_image_size;
        for(int y = 0, idx = 0; y < m_h; ++y) {
            for(int x = 0; x < m_w; ++x) {
                if(x+d>=m_w) {idx++;continue;}
                mcost[idx] = qing_get_mcost_tad(m_bgr_l+(idx+d)*3, m_bgr_r+idx*3, 3);
                idx++;
            }
        }
    }

# if 0
    qing_create_dir("matching-cost");
    for(int d = 0; d < m_disp_range; ++d) {
        string out_txt_file = "./matching-cost/mcost_l_" + qing_int_2_string(d) + ".txt";
        string out_jpg_file = "./matching-cost/mcost_l_" + qing_int_2_string(d) + ".jpg";
        float * mcost = m_mcost_r + d * m_image_size;
        qing_save_mcost_txt(out_txt_file, mcost, m_image_size, m_w);
        qing_save_mcost_jpg(out_jpg_file, mcost, m_w, m_h);
    }
# endif
}


void qing_stereo::aggregate_mcost_vol() {

    //    m_filtered_mcost_l = new float[m_image_size * m_disp_range];
    //    memset(m_filtered_mcost_l, 0, sizeof(m_filtered_mcost_l));

    for(int d = 0; d < m_disp_range; ++d) {
        float * mcost = m_mcost_l + d * m_image_size;
        float * temp = new float[m_image_size];

        memcpy(temp, mcost, sizeof(float)*m_image_size);
        memset(mcost, 0, sizeof(float)*m_image_size);
        QingTimer timer;
        qing_approximated_bilateral_filter(mcost, temp, m_bgr_l, m_w, m_h, m_sigma_range, m_sigma_spatial);
        //qing_bilateral_filter(mcost, temp, m_bgr_l, m_w, m_h, m_sigma_range, m_sigma_spatial);

        cout << "d = " << d << ", duration = " << timer.duration()*1000 << " ms" << endl;
# if 0
        qing_create_dir("matching-cost");
        string out_jpg_file = "./matching-cost/filtered_mcost_l_" + qing_int_2_string(d) + ".jpg";
        qing_save_mcost_jpg(out_jpg_file, mcost, m_w, m_h);
# endif
    }
}

void qing_stereo::mcost_to_disp(const int scale) {
    m_disp_l = new unsigned char[m_image_size];
    memset(m_disp_l, 0, sizeof(unsigned char)*m_image_size);
    for(int y = 0, idx = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {

            float min_mcost = 65536;
            int min_d = 0;

            for(int d = 1; d < m_disp_range; ++d) {
                float mcost = *(m_mcost_l + d * m_image_size + idx);
                if(mcost < min_mcost) {
                    min_mcost = mcost;
                    min_d = d * scale;
                }
            }
            m_disp_l[idx] = min_d;
            idx++;
        }
    }

# if 1
    Mat left_disp(m_h, m_w, CV_8UC1);
    memcpy(left_disp.data, m_disp_l, sizeof(unsigned char)*m_image_size);
    imwrite("./left.png", left_disp);
# endif
}
