#include "../../../Qing/qing_common.h"
#include "qing_stereo.h"

int main(int argc, char * argv[]) {

    const string imagename_l = "../input/half_wood1_L.png";
    const string imagename_r = "../input/half_wood1_R.png";

    int d = 70;
    float sigma_range = 0.08;
    float sigma_spatial = 0.03;

    qing_stereo stereo;
    stereo.load_image(imagename_l, imagename_r);
    stereo.set_params(d, sigma_range, sigma_spatial);

    stereo.compute_mcost_vol_l();
    stereo.aggregate_mcost_vol();
//    stereo.directional_aggregate_mcost_vol();
    stereo.mcost_to_disp(255/d);
//    stereo.compute_mcost_vol_r();

//    cv:: SiftFeatureDetector sift_features ;

}
