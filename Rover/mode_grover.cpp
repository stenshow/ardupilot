#include "mode.h"
#include "Rover.h"

void ModeGrover::update()
{

/// add code ///------------------------------------------------------------------
    // get (in)exclusion polygon
    uint16_t nums; // ジオフェンス座標の数
    uint16_t index; //
    double point_x[12]; // ジオフェンスx座標
    double point_y[12]; // ジオフェンスy座標

    Vector2f *vp; // polygonのベクトル
    index = 0;
    int i = 0;

    vp = rover.g2.fence.polyfence().get_inclusion_polygon(index,nums);
    //gcs().send_text(MAV_SEVERITY_CRITICAL,"exclusion_poly_num : %d",nums);
    //gcs().send_text(MAV_SEVERITY_CRITICAL,"index : %d",index);

    
    while(i<nums) {
        point_x[i] = (vp + i)->x;
        point_y[i] = (vp + i)->y;
        //gcs().send_text(MAV_SEVERITY_CRITICAL,"vector_poly_x : %f", point_x[i]);
        //gcs().send_text(MAV_SEVERITY_CRITICAL,"vector_poly_y : %f", point_y[i]);
        //gcs().send_text(MAV_SEVERITY_CRITICAL,"i : %d", i);
        i = i + 1;
    }

    // 重心算出
    double sum_x = 0; // x座標の合計
    double sum_y = 0; // y座標の合計
    double gc_x = 0; // 重心のx座標
    double gc_y = 0; // 重心のy座標

    i = 0; // iをリセット
    while(i<nums) {
        sum_x = sum_x + point_x[i];
        sum_y = sum_y + point_y[i];
        i = i + 1;
        //gcs().send_text(MAV_SEVERITY_CRITICAL,"sum_x : %f", point_x[i]);
        //gcs().send_text(MAV_SEVERITY_CRITICAL,"sum_y : %f", point_y[i]);
    }

    gc_x = (sum_x/nums)*0.01;
    gc_y = (sum_y/nums)*0.01;

///// calculation coordinate function main /////
    uint16_t lap_num; // lap number
    uint16_t move_rover = 120; // distance by rover [cm]

    ///--- calculation  of lap number ---///
    // x&y coordinate values
    // short side 
    double x_diff = fabsf(point_x[0] - point_x[1]); // length of side_x
    double y_diff = fabsf(point_y[1] - point_y[2]); // length of side_y

    //gcs().send_text(MAV_SEVERITY_CRITICAL,"x diff : %f",x_diff);
    //gcs().send_text(MAV_SEVERITY_CRITICAL,"y diff : %f",y_diff);

    
    int short_side = MIN(x_diff,y_diff); // minimum of length
    short_side = short_side / 2;
    lap_num = short_side / move_rover;

    ///--- calculation target coordinates ---///
    // array of target coordinates
    //  b             c
    //  o - - - - - - o
    //  | o         o |
    //  |             |
    //  |             | 
    //  |  o        o |
    //  o - - - - - - o
    //  a             d
    std::vector<double> ax(lap_num),bx(lap_num),cx(lap_num),dx(lap_num);
    std::vector<double> ay(lap_num),by(lap_num),cy(lap_num),dy(lap_num);

    // from each of coordinate to center
    double dis_x = fabsf(point_x[0] - gc_x*100.0); // 中心までの距離
    double dis_y = fabsf(point_y[0] - gc_y*100.0);
    double move_x = dis_x / (lap_num + 1); // 各座標の移動距離
    double move_y = dis_y/ (lap_num + 1);

    for(int j=0;j<lap_num;j++) {
        
        ax[j] = point_x[0] + move_x*(j+1); 
        ay[j] = point_y[0] + move_y*(j+1);

        bx[j] = point_x[1] - move_x*(j+1);
        by[j] = point_y[1] + move_y*(j+1);
    
        cx[j] = point_x[2] - move_x*(j+1);
        cy[j] = point_y[2] - move_y*(j+1);
    
        dx[j] = point_x[3] + move_x*(j+1);
        dy[j] = point_y[3] - move_y*(j+1);


        /*
        gcs().send_text(MAV_SEVERITY_CRITICAL,"ax : %f",ax[j]);
        gcs().send_text(MAV_SEVERITY_CRITICAL,"ay : %f",ay[j]);

        gcs().send_text(MAV_SEVERITY_CRITICAL,"bx : %f",bx[j]);
        gcs().send_text(MAV_SEVERITY_CRITICAL,"by : %f",by[j]);

        gcs().send_text(MAV_SEVERITY_CRITICAL,"cx : %f",cx[j]);
        gcs().send_text(MAV_SEVERITY_CRITICAL,"cy : %f",cy[j]);

        gcs().send_text(MAV_SEVERITY_CRITICAL,"dx : %f",dx[j]);
        gcs().send_text(MAV_SEVERITY_CRITICAL,"dy : %f",dy[j]);
        */
    }

    ///// move to target location /////
    // get current location
    Vector3f ahrs_pos_ned;
    float x_cur, y_cur;
    ahrs.get_relative_position_NED_origin(ahrs_pos_ned);
    x_cur = ahrs_pos_ned[0];
    y_cur = ahrs_pos_ned[1];
    x_cur = x_cur; 
    y_cur = y_cur;

    // 変数定義
    float deg = 0.0;
    float d_east = 0.0;
    float d_north = 0.0;
    float distance = 0.0;
    float heading = 0.0;
    //float turn_rate = -4500.0;
    //float lat_accel = 1000.0;

    /// グルグル
    static int count = 0; // 配列を切り替える
    static int k = 0; // 周回範囲を切り替える
    int turn_num = 1; // from current loc to target loc

    
    if(count == 0 && k < lap_num - 1) {
        d_north = 0.01*ax[k] - x_cur;
        d_east = 0.01*ay[k] - y_cur;
        deg = degrees( atan2f( d_east, d_north ));
        distance = sqrtf((d_north*d_north) + (d_east*d_east));
        heading = deg*100.0;  /// *100.0
        calc_steering_to_heading(heading);
        calc_throttle(calc_speed_nudge(0.5, is_negative(0.5)), true);
        //calc_steering_from_lateral_acceleration(lat_accel, true);
        //calc_steering_from_turn_rate(turn_rate);
        if(distance < turn_num) count++;
    } else if(count == 1 && k < lap_num - 1) {
        d_north = 0.01*bx[k] - x_cur;
        d_east = 0.01*by[k] - y_cur;
        deg = degrees( atan2f( d_east, d_north ));
        distance = sqrtf((d_north*d_north) + (d_east*d_east));
        heading = deg*100.0;
        calc_steering_to_heading(heading);
        calc_throttle(calc_speed_nudge(0.5, is_negative(0.5)), true);
        if(distance < turn_num) count++;
    } else if(count == 2 && k < lap_num - 1) {
        d_north = 0.01*cx[k] - x_cur;
        d_east = 0.01*cy[k] - y_cur;
        deg = degrees( atan2f( d_east, d_north ));
        distance = sqrtf((d_north*d_north) + (d_east*d_east));
        heading = deg*100.0;
        calc_steering_to_heading(heading);
        calc_throttle(calc_speed_nudge(0.5, is_negative(0.5)), true);
        if(distance < turn_num) count++;
    } else if(count == 3 && k < lap_num - 1) {
        d_north = 0.01*dx[k] - x_cur;
        d_east = 0.01*dy[k] - y_cur;
        deg = degrees( atan2f( d_east, d_north ));
        distance = sqrtf((d_north*d_north) + (d_east*d_east));
        heading = deg*100.0;
        calc_steering_to_heading(heading);
        calc_throttle(calc_speed_nudge(0.5, is_negative(0.5)), true);
        if(distance < turn_num) count++;
    } else if(count == 4 && k < lap_num - 1) {
        d_north = 0.01*ax[k] - x_cur;
        d_east = 0.01*ay[k] - y_cur;
        deg = degrees( atan2f( d_east, d_north ));
        distance = sqrtf((d_north*d_north) + (d_east*d_east));
        heading = deg*100.0;
        calc_steering_to_heading(heading);
        calc_throttle(calc_speed_nudge(0.5, is_negative(0.5)), true);
        if(distance < turn_num) {
            count = 0;
            k++;
        }
    }

    if(k == lap_num - 1) {
        d_north = gc_x - x_cur;
        d_east = gc_y - y_cur;
        deg = degrees( atan2f( d_east, d_north ));
        distance = sqrtf((d_north*d_north) + (d_east*d_east));
        heading = deg*100.0;
        // 制御する部分
        calc_steering_to_heading(heading);
        calc_throttle(calc_speed_nudge(0.5, is_negative(0.5)), true);
        if(distance < turn_num) {
            d_north = 0.01*ax[0] - x_cur;
            d_east = 0.01*ay[0] - y_cur;
            deg = degrees( atan2f( d_east, d_north ));
            distance = sqrtf((d_north*d_north) + (d_east*d_east));
            heading = deg*100.0;
            calc_steering_to_heading(heading);
            calc_throttle(calc_speed_nudge(0.5, is_negative(0.5)), true);
        }
    }

}

bool ModeGrover::requires_velocity() const
{
    return !g2.motors.have_skid_steering();
}

// sailboats in acro mode support user manually initiating tacking from transmitter
void ModeGrover::handle_tack_request()
{
    rover.g2.sailboat.handle_tack_request_acro();
}


