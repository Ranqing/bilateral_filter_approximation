#include "../../../Qing/qing_common.h"
#include "qing_stereo.h"

int main(int argc, char * argv[]) {

//        const string imagename_l = "q_wood1_L.png";
//        const string imagename_r = "q_wood1_R.png";
//        const string folder = "../input/";

    const string folder = "../input/teddy/";
    const string imagename_l = "teddy_L.png";
    const string imagename_r = "teddy_R.png";

    int d = 70;
    float sigma_range = 0.08;
    float sigma_spatial = 0.03;
    int wnd = 31;

    //for teddy:
    //sigma_range = 10;
    //sigma_spatial = 11

    qing_stereo stereo;
    //order can't be adjusted
    stereo.set_params(d, sigma_range, sigma_spatial);
    stereo.load_image(folder+imagename_l, folder+imagename_r);
    stereo.malloc();

    stereo.compute_mcost_vol_l();
    //stereo.aggregate_mcost_vol(wnd);
    stereo.directional_aggregate_mcost_vol(wnd);
    stereo.mcost_to_disp("d_disp_bf_" + imagename_l, 255/d);
    //    stereo.compute_mcost_vol_r();


}
