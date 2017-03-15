#include "qing_stereo.h"

#include "../../../Qing/qing_macros.h"
#include "../../../Qing/qing_string.h"
#include "../../../Qing/qing_matching_cost.h"
#include "../../../Qing/qing_bilateral_filter.h"
#include "../../../Qing/qing_dir.h"
#include "../../../Qing/qing_timer.h"
#include "../../../Qing/qing_image.h"

// unsigned char * m_bgr_l, * m_bgr_r;
//float * m_mcost_l, * m_mcost_r;
//float * m_filtered_mcost_l, * m_filtered_mcost_r;
//float * m_range_table, * m_spatial_table;
//double m_sigma_range, m_sigma_spatial;
//unsigned char * m_disp_l, * m_disp_r;


qing_stereo::qing_stereo(): m_bgr_l(0), m_bgr_r(0), m_mcost_l(0), m_mcost_r(0), m_filtered_mcost_l(0), m_filtered_mcost_r(0),
    m_range_table(0), m_spatial_table(0), m_disp_l(0), m_disp_r(0)
{
}

qing_stereo::~qing_stereo() {
    if(m_bgr_l!=0) {delete[] m_bgr_l;m_bgr_l=0;}
    if(m_bgr_r!=0) {delete[] m_bgr_r;m_bgr_r=0;}
    if(m_mcost_l!=0) {delete[] m_mcost_l;m_mcost_l=0;}
    if(m_mcost_r!=0) {delete[] m_mcost_r;m_mcost_r=0;}
    if(m_filtered_mcost_l!=0) {delete[] m_filtered_mcost_l; m_filtered_mcost_l = 0; }
    if(m_filtered_mcost_r!=0) {delete[] m_filtered_mcost_r; m_filtered_mcost_r = 0; }
    if(m_range_table!=0) {delete[] m_range_table; m_range_table = 0; }
    if(m_spatial_table!=0) {delete[] m_spatial_table; m_spatial_table = 0; }
    if(m_disp_l!=0) {delete[] m_disp_l; m_disp_l = 0; }
    if(m_disp_r!=0) {delete[] m_disp_r; m_disp_r = 0; }

}

void qing_stereo::set_params(const int disp_range, const float sigma_range, const float sigma_spatial) {
    m_disp_range = disp_range;
    m_sigma_range = sigma_range;
    m_sigma_spatial = sigma_spatial;
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

    cout << "finish loading images"  << endl;
    cout << m_disp_range << " x " << m_h << " x " << m_w << endl;

    Mat gray_l, gray_r;
    cvtColor(mat_l, gray_l, CV_BGR2GRAY);
    cvtColor(mat_r, gray_r, CV_BGR2GRAY);
    m_gray_l = new unsigned char[m_image_size];
    m_gray_r = new unsigned char[m_image_size];
    memcpy(m_gray_l, gray_l.data, sizeof(unsigned char)*m_image_size);
    memcpy(m_gray_r, gray_r.data, sizeof(unsigned char)*m_image_size);
    m_census_l = new unsigned char[m_image_size];
    m_census_r = new unsigned char[m_image_size];
    qx_census_transform_3x3(m_census_l, m_gray_l, m_h, m_w);
    qx_census_transform_3x3(m_census_r, m_gray_r, m_h, m_w);
    cout << "3x3 census transform done.." << endl;
}

void qing_stereo::get_weighted_table() {
    m_range_table = qing_get_range_weighted_table(m_sigma_range, QING_FILTER_INTENSITY_RANGE);
   m_spatial_table = qing_get_spatial_weighted_table(m_sigma_spatial, QING_FILTER_SPATIAL_RANGE);

    cout << QING_DEBUG_FLAG_STRING << "\t finish weighted table computing..." << endl;
}

void qing_stereo::malloc() {

    m_sigma_range = m_sigma_range * QING_FILTER_INTENSITY_RANGE;
    m_sigma_spatial = m_sigma_spatial * min(m_w, m_h);
    cout << "sigma_range = " << m_sigma_range <<  "\t sigma_spatial = " << m_sigma_spatial << endl;
    get_weighted_table();

    m_mcost_l = new float[m_total_size];
    if(m_mcost_l == 0) {
        cerr << "failed to new left matching cost.. " << endl;
        exit(-1);
    }
    std::fill(m_mcost_l, m_mcost_l + m_total_size, 0);
    m_filtered_mcost_l = new float[m_total_size];
    if(m_filtered_mcost_l == 0) {
        cerr << "failed to new left filtered matching cost.. " << endl;
        exit(-1);
    }
    std::fill(m_filtered_mcost_l, m_filtered_mcost_l + m_total_size, 0);

# if STEREO_RIGHT
    m_mcost_r = new float[m_total_size];
    if(m_mcost_r == 0) {
        cerr << "failed to new right matching cost.." << endl;
        exit(-1);
    }
    m_filtered_mcost_r = new float[m_total_size];
    if(m_filtered_mcost_r == 0) {
        cerr << "failed to new right filtered matching cost.. " << endl;
        exit(-1);
    }
# endif
}

// 0 is the invalid matching cost
void qing_stereo::compute_mcost_vol_l(){
    for(int d = 0; d < m_disp_range; ++d) {
        float * mcost = m_mcost_l + d * m_image_size;
        for(int y = 0, idx = 0; y < m_h; ++y) {
            for(int x = 0; x < m_w; ++x) {
                if(x-d<0) mcost[idx] = 0.f; /*QING_TAD_TRUNCATED;*/
                else  mcost[idx] = qing_get_mcost_tad( m_bgr_l+idx*3, m_bgr_r+(idx-d)*3, 3);
                idx++;
            }
        }
    }
    cout << "finish computing raw matching cost vol..." << endl;

# if 0
    qing_create_dir("matching-cost");
    for(int d = 0; d < m_disp_range; ++d) {
        string out_txt_file = "./matching-cost/mcost_l_" + qing_int_2_string(d) + ".txt";
        string out_jpg_file = "./matching-cost/mcost_l_" + qing_int_2_string(d) + ".jpg";
        float * mcost = m_mcost_l + d * m_image_size;
        qing_save_mcost_txt(out_txt_file, mcost, m_image_size, m_w);
        qing_save_mcost_jpg_inf(out_jpg_file, mcost, m_w, m_h);
    }
# endif
}

void qing_stereo::compute_mcost_vol_r(){
    for(int d = 0; d < m_disp_range; ++d) {
        float * mcost = m_mcost_r + d * m_image_size;
        for(int y = 0, idx = 0; y < m_h; ++y) {
            for(int x = 0; x < m_w; ++x) {
                if(x+d>=m_w) mcost[idx] = 0.f; /*QING_TAD_TRUNCATED;*/
                else mcost[idx] = qing_get_mcost_tad(m_bgr_l+(idx+d)*3, m_bgr_r+idx*3, 3);
                idx++;
            }
        }
    }
    cout << "finish computing raw matching cost vol... in right" << endl;

# if 0
    qing_create_dir("matching-cost");
    for(int d = 0; d < m_disp_range; ++d) {
        string out_txt_file = "./matching-cost/mcost_r_" + qing_int_2_string(d) + ".txt";
        string out_jpg_file = "./matching-cost/mcost_r_" + qing_int_2_string(d) + ".jpg";
        float * mcost = m_mcost_r + d * m_image_size;
        qing_save_mcost_txt(out_txt_file, mcost, m_image_size, m_w);
        qing_save_mcost_jpg_inf(out_jpg_file, mcost, m_w, m_h);
    }
# endif
}

void qing_stereo::compute_mcost_vol_census_l() {

    for(int d = 0; d < m_disp_range; ++d) {
        float * mcost = m_mcost_l + d * m_image_size;
        for(int y = 0, idx = 0; y < m_h; ++y) {
            for(int x = 0; x < m_w; ++x) {
                if(x-d>=m_w || x-d<0) mcost[idx] = 0.f;
                else {
                    mcost[idx] = qx_hamming_distance(m_census_l[idx], m_census_r[idx-d]);
#if 1
                    if(y != 0)       {  //y-1
                        float temp_mcost = qx_hamming_distance(m_census_l[idx], m_census_r[idx-d-m_w]);
                        if(mcost[idx] > temp_mcost) mcost[idx] = temp_mcost;
                    }
                    if(y != m_h - 1) { //y+1
                        float temp_mcost = qx_hamming_distance(m_census_l[idx], m_census_r[idx-d+m_w]);
                        if(mcost[idx] > temp_mcost) mcost[idx] = temp_mcost;
                    }
#endif
                }
                idx++;
            }
        }
    }
    cout << "finish computing census transform matching cost vol..." << endl;
# if 1
    qing_create_dir("census-matching-cost");
    for(int d = 0; d < m_disp_range; ++d) {
        string out_jpg_file = "./census-matching-cost/census_l_" + qing_int_2_string(d) + ".jpg";
        float * mcost = m_mcost_l + d * m_image_size;
        qing_save_mcost_jpg(out_jpg_file,  mcost, m_w, m_h);
    }
# endif
}

void qing_stereo::compute_mcost_vol_census_r() {

    for(int d = 0; d < m_disp_range; ++d) {
        float * mcost = m_mcost_r + d * m_image_size;
        for(int y = 0, idx = 0; y < m_h; ++y) {
            for(int x = 0; x < m_w; ++x) {
                if(x+d>=m_w || x-d<0) mcost[idx] = 0.f;
                else
                    mcost[idx] = qx_hamming_distance(m_census_l[idx+d], m_census_r[idx]);
                idx++;
            }
        }
    }
    cout << "finishing computing census transform matching cost vol...in right.." << endl;
# if 1
    qing_create_dir("census-matching-cost");
    for(int d = 0; d < m_disp_range; ++d) {
        string out_jpg_file = "./census-matching-cost/census_r_" + qing_int_2_string(d) + ".jpg";
        float * mcost = m_mcost_r + d * m_image_size;
        qing_save_mcost_jpg(out_jpg_file, mcost, m_w, m_h);
    }
# endif
}

void qing_stereo::aggregate_mcost_vol(const int wnd) {

    cout << "aggregate_mcost_vol..." << endl;
    cout << "sigma_range = "   << m_sigma_range << endl;
    cout << "sigma_spatial = " << m_sigma_spatial << endl;
    cout << "wnd_size = "      << wnd << endl;

    QingTimer timer;
    memcpy(m_filtered_mcost_l, m_mcost_l, m_total_size * sizeof(float));
    for(int d = 0; d < m_disp_range; ++d) {

        timer.restart();
        float * mcost = m_filtered_mcost_l + d * m_image_size;
        float * temp = new float[m_image_size];

        memcpy(temp, mcost, sizeof(float)*m_image_size);
        memset(mcost, 0, sizeof(float)*m_image_size);

        qing_approximated_bilateral_filter(mcost, temp, m_bgr_l, m_w, m_h, wnd, m_range_table, m_spatial_table);
        // qing_bilateral_filter(mcost, temp, m_bgr_l, m_w, m_h, m_sigma_range, m_sigma_spatial);

        cout << "d = " << d << ", duration = " << timer.duration()*1000 << " ms" << endl;
# if 0
        qing_create_dir("matching-cost");
        string out_txt_file = "./matching-cost/approx_filtered_mcost_l_" + qing_int_2_string(d) + ".txt";
        qing_save_mcost_txt(out_txt_file, mcost, m_w*m_h, m_w);
        string out_jpg_file = "./matching-cost/approx_filtered_mcost_l_" + qing_int_2_string(d) + ".jpg";
        qing_save_mcost_jpg_inf(out_jpg_file, mcost, m_w, m_h);
# endif
    }
}

void qing_stereo::directional_aggregate_mcost_vol(const int wnd) {

    cout << "directional_aggregate_mcost_vol..." << endl;
    cout << "sigma_range = "   << m_sigma_range << endl;
    cout << "sigma_spatial = " << m_sigma_spatial << endl;
    cout << "wnd_size = "      << wnd << endl;

# if 1
    int x_len = 21;
    float x_step = 1.0f/(x_len/2);
    float * x_directions = qing_get_directions(x_step, x_len);
    cout << "x_directions calculation done..." << endl;
    for(int ii = 0; ii < x_len; ++ii)
        cout << x_directions[ii] << ' ';
    cout << endl;
# endif
# if 1
    int y_len = 21;
    float y_step = 1.0f/(y_len/2);
    float * y_directions = qing_get_directions(y_step, y_len);
    cout << "y_directions calculation done..." << endl;
    for(int ii = 0; ii < y_len; ++ii)
        cout << y_directions[ii] << ' ';
    cout << endl;
# endif

    std::fill(m_filtered_mcost_l, m_filtered_mcost_l + m_total_size, QING_MAX_MCOST); //min_mcost_xy
    float * min_mcost_x = new float[m_total_size];
    if(0==min_mcost_x) {
        cerr << "failed to new minimum matching cost for x-direction filtering.." << endl;
        exit(0);
    }
    std::fill(min_mcost_x, min_mcost_x + m_total_size, QING_MAX_MCOST);

    int offset = (int)wnd *0.5;

    QingTimer timer;
    double * weights = new double[wnd];
    double sum, sum_div;
    float x_dir, y_dir;
    unsigned char * ptr_bgr_c, * ptr_bgr_k;
    unsigned char b_c, g_c, r_c;
    int delta_b, delta_g, delta_r;

    cout << "horizontal filtering start...." << endl;
    for(int d = 0; d < m_disp_range; ++d) {
        timer.restart();
        float * out = min_mcost_x + d * m_image_size;

        for(int y = 0; y < m_h; ++y) {
            int idy = y * m_w;
            for(int x = 0; x < m_w; ++x) {
                int idx = idy + x;

                ptr_bgr_c = m_bgr_l + 3 * idx -1;
                b_c = *(++ptr_bgr_c);
                g_c = *(++ptr_bgr_c);
                r_c = *(++ptr_bgr_c);

                //update out[idx]

                std::fill(weights, weights+wnd, 0.0);
                for(int k = -offset, tk = 0; k <= offset; ++tk, ++k) {
                    if(x+k < 0 || x+k >= m_w) continue;
                    int idk = idx + k;
                    ptr_bgr_k = m_bgr_l + 3 * idk - 1;
                    delta_b = abs(*(++ptr_bgr_k) - b_c);
                    delta_g = abs(*(++ptr_bgr_k) - g_c);
                    delta_r = abs(*(++ptr_bgr_k) - r_c);

                    int delta_bgr = (int)(0.3333 * (delta_b + delta_g + delta_r));
                    weights[tk] = m_range_table[delta_bgr] * m_spatial_table[abs(k)];

                    //weights[tk] = m_range_table[delta_b] * m_range_table[delta_g] * m_range_table[delta_r] * m_spatial_table[abs(k)];
                }

                for(int i = 0; i < x_len; ++i) {
                    x_dir = x_directions[i];
                    sum = 0.0;  sum_div = 0.0;

                    for(int k = -offset, tk = 0; k <= offset; ++tk, ++k) {
                        if(x+k < 0 || x+k >= m_w) continue;
                        int idk = idx + k;
                        int k_d = x_dir * k + d;
                        k_d=max(0,k_d);
                        k_d=min(m_disp_range-1,k_d);
                        //if(k_d < 0 || k_d >= m_disp_range) continue;

                        sum += weights[tk] * (*(m_mcost_l + k_d * m_image_size + idk)) ;
                        //sum += weights[tk] * (*(m_mcost_l + d * m_image_size + idk)) ;

                        sum_div += weights[tk];
                    }

                    if(sum_div >= 0.000001) sum/=sum_div;
                    if(sum < out[idx]) out[idx] = sum;
                }
            }
        }
        cout << "d = " << d << "\tduration = " << timer.duration() * 1000 << " ms" << endl;
# if 0
        qing_create_dir("matching-cost");
        string savefile = "matching-cost/x_filtered_mcost_" + qing_int_2_string(d) + ".jpg";
        qing_save_mcost_jpg_inf(savefile, out, m_w, m_h);
# endif
    }
    cout << "horizontal filtering done.." << endl;

 # if 1
    Mat x_disp_mat = Mat::zeros(m_h, m_w, CV_8UC1);
    unsigned char * ptr = x_disp_mat.ptr<unsigned char>(0);
    qing_wta_disparity(ptr, min_mcost_x, m_disp_range, m_h, m_w, 255/m_disp_range);
    string file = "../disp_in_x.png";
    imwrite(file, x_disp_mat); cout << "saving " << file << endl;
 # endif

    cout << "vertical filtering start..." << endl;
    for(int d = 0; d < m_disp_range; ++d) {
        timer.restart();
        float * out = m_filtered_mcost_l + d * m_image_size;

        for(int y = 0; y < m_h; ++y) {
            int idy = y * m_w;
            for(int x = 0; x < m_w; ++x) {
                int idx = idy + x;

                ptr_bgr_c = m_bgr_l + 3 * idx -1;
                b_c = *(++ptr_bgr_c);
                g_c = *(++ptr_bgr_c);
                r_c = *(++ptr_bgr_c);

                //update out[idx]

                std::fill(weights, weights+wnd, 0.0);
                for(int k = -offset, tk = 0; k <= offset; ++tk, ++k) {
                    if(y+k < 0 || y+k >= m_h) continue;
                    int idk = idx + k * m_w;
                    ptr_bgr_k = m_bgr_l + 3 * idk - 1;
                    delta_b = abs(*(++ptr_bgr_k) - b_c);
                    delta_g = abs(*(++ptr_bgr_k) - g_c);
                    delta_r = abs(*(++ptr_bgr_k) - r_c);

                    int delta_bgr = (int)(0.3333 * (delta_b + delta_g + delta_r));
                    weights[tk] = m_range_table[delta_bgr] * m_spatial_table[abs(k)];

                    // weights[tk] = m_range_table[delta_b] * m_range_table[delta_g] * m_range_table[delta_r] * m_spatial_table[abs(k)];
                }

                for(int i = 0; i < y_len; ++i) {
                    y_dir = y_directions[i];
                    sum = 0.0;  sum_div = 0.0;

                    for(int k = -offset, tk = 0; k <= offset; ++tk, ++k) {
                        if(y+k < 0 || y+k >= m_h) continue;
                        int idk = idx + k * m_w;
                        int k_d = y_dir * k + d;
                        k_d=max(0,k_d);
                        k_d=min(m_disp_range-1,k_d);

                        //                        if(k_d < 0 || k_d >= m_disp_range) continue;

                        sum += weights[tk] * (*(min_mcost_x + k_d * m_image_size + idk)) ;
                        sum_div += weights[tk];
                    }

                    if(sum_div >= 0.000001) sum/=sum_div;
                    if(sum < out[idx]) out[idx] = sum;
                }
            }
        }

        cout << "d = " << d << "\tduration = " << timer.duration() * 1000 << " ms" << endl;
# if 0
        qing_create_dir("matching-cost");
        string savefile = "matching-cost/y_filtered_mcost_" + qing_int_2_string(d) + ".jpg";
        qing_save_mcost_jpg_inf(savefile, out, m_w, m_h);
# endif
    }
    cout << "vertical filtering done.." << endl;

# if 1
   Mat y_disp_mat = Mat::zeros(m_h, m_w, CV_8UC1);
   ptr = y_disp_mat.ptr<unsigned char>(0);
   qing_wta_disparity(ptr, m_filtered_mcost_l, m_disp_range, m_h, m_w, 255/m_disp_range);
   file = "../disp_in_y.png";
   imwrite(file, y_disp_mat); cout << "saving " << file << endl;
# endif
}

void qing_stereo::mcost_to_disp(const string file_suffix, const int scale) {
    m_disp_l = new unsigned char[m_image_size];
    memset(m_disp_l, 0, sizeof(unsigned char)*m_image_size);
    for(int y = 0, idx = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {

            float min_mcost = 10000.f;
            int min_d = 0;

            for(int d = 1; d < m_disp_range; ++d) {
                //              float mcost = *(m_mcost_l + d * m_image_size + idx);
                float mcost = *(m_filtered_mcost_l + d * m_image_size + idx);
                if(mcost < min_mcost) {
                    min_mcost = mcost;
                    min_d = d * scale;
                }
            }
            //            if(x-min_d>=0) m_disp_l[idx] = min_d;
            //            else m_disp_l[idx] = 0;
            m_disp_l[idx] = min_d;
            idx++;
        }
    }

# if 1
    Mat left_disp(m_h, m_w, CV_8UC1);
    memcpy(left_disp.data, m_disp_l, sizeof(unsigned char)*m_image_size);
    string out_file = "./left_" + file_suffix ;
    imwrite(out_file, left_disp); cout << "saving " << out_file << endl;
# endif
}
