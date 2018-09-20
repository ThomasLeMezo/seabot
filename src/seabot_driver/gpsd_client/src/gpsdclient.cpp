#include "gpsdclient.h"

bool GPSDClient::start() {
  gps_fix_pub = node.advertise<gpsd_client::GPSFix>("fix", 1);

  privnode.getParam("use_gps_time", use_gps_time);
  privnode.getParam("check_fix_by_variance", check_fix_by_variance);
  privnode.param("frame_id", frame_id, frame_id);

  gps = new gpsmm("localhost", DEFAULT_GPSD_PORT);

  if(gps->stream(WATCH_ENABLE | WATCH_JSON) == nullptr){
    ROS_ERROR("[GPSD_Client] Failed to open GPSd");
    return false;
  }
  else{
    ROS_DEBUG("[GPSD_Client] GPSd opened");
    return true;
  }
}

void GPSDClient::step() {
  if (!gps->waiting(2e6)) // us ?
    return;

  gps_data_t *p;
  if((p = gps->read())==nullptr){
    ROS_WARN("[GPSD_Client] Error reading gpsd");
  }
  else{
    process_data(p);
  }
}

void GPSDClient::stop() {
  gps->~gpsmm();
}

void GPSDClient::process_data(struct gps_data_t* p) {
  if (p->online==0){
    ROS_INFO("[GPSD_Client] Online = 0");
    return;
  }

  bool new_fix_state = (p->fix.mode>=MODE_2D)?true:false;
  if(new_fix_state ||(m_last_fix_state!=new_fix_state)){
    process_data_gps(p);
    m_last_fix_state =  new_fix_state;
  }
  else if(p->fix.mode==MODE_NOT_SEEN){
    ROS_INFO("[GPSD_Client] fix = MODE_NOT_SEEN");
  }
}

void GPSDClient::process_data_gps(struct gps_data_t* p) {
  ros::Time time = ros::Time::now();

  gpsd_client::GPSFix fix;
  gpsd_client::GPSStatus status;

  status.header.stamp = time;
  fix.header.stamp = time;
  fix.header.frame_id = frame_id;

  status.satellites_used = p->satellites_used;

  status.satellite_used_prn.resize(status.satellites_used);
  for (int i = 0; i < status.satellites_used; ++i) {
    status.satellite_used_prn[i] = p->skyview[i].used;
  }

  status.satellites_visible = p->satellites_visible;

  status.satellite_visible_prn.resize(status.satellites_visible);
  status.satellite_visible_z.resize(status.satellites_visible);
  status.satellite_visible_azimuth.resize(status.satellites_visible);
  status.satellite_visible_snr.resize(status.satellites_visible);

  for (int i = 0; i < status.satellites_visible; ++i) {
    status.satellite_visible_prn[i] = p->skyview[i].PRN;
    status.satellite_visible_z[i] = p->skyview[i].elevation;
    status.satellite_visible_azimuth[i] = p->skyview[i].azimuth;
    status.satellite_visible_snr[i] = p->skyview[i].ss;
  }

  if(p->status != STATUS_NO_FIX) {
    status.status = p->fix.mode; // FIXME: gpsmm puts its constants in the global
    // namespace, so `GPSStatus::STATUS_FIX' is illegal.

    // STATUS_DGPS_FIX was removed in API version 6 but re-added afterward
#if GPSD_API_MAJOR_VERSION != 6
    if (p->status & STATUS_DGPS_FIX)
      status.status |= 18; // same here
#endif

    fix.time = p->fix.time;
    fix.latitude = p->fix.latitude;
    fix.longitude = p->fix.longitude;
    fix.altitude = p->fix.altitude;
    fix.track = p->fix.track;
    fix.speed = p->fix.speed;
    fix.climb = p->fix.climb;

    fix.pdop = p->dop.pdop;
    fix.hdop = p->dop.hdop;
    fix.vdop = p->dop.vdop;
    fix.tdop = p->dop.tdop;
    fix.gdop = p->dop.gdop;

    fix.err = p->epe;
    fix.err_vert = p->fix.epv;
    fix.err_track = p->fix.epd;
    fix.err_speed = p->fix.eps;
    fix.err_climb = p->fix.epc;
    fix.err_time = p->fix.ept;
  }
  else {
    status.status = -1; // STATUS_NO_FIX
  }
  fix.status = status;

  gps_fix_pub.publish(fix);
}
