#include "gpsdclient.h"
#include "ros/ros.h"

bool GPSDClient::start() {
  gps_fix_pub = node.advertise<gpsd_client::GPSFix>("fix", 1);

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
  gps->clear_fix();
}

void GPSDClient::stop() {
  gps->stream(WATCH_DISABLE);
  gps->~gpsmm();
}

void GPSDClient::step() {
  struct gps_data_t *p;
  if (!gps->waiting(50000000)) // us ? => 1s
    return;

  if((p = gps->read())==NULL)
    ROS_WARN("[GPSD_Client] Error reading gpsd");
  else
    process_data_gps(p);
}



void GPSDClient::process_data_gps(struct gps_data_t* p) {
  ros::Time time = ros::Time::now();

  m_fix.header.stamp = time;
  m_fix.header.frame_id = frame_id;

  m_fix.status = p->fix.mode; // FIXME: gpsmm puts its constants in the global
  m_fix.time = p->fix.time.tv_sec+p->fix.time.tv_nsec*1e-9;

  if(p->fix.mode >= MODE_2D) {
    m_fix.latitude = p->fix.latitude;
    m_fix.longitude = p->fix.longitude;

    m_fix.altitude = p->fix.altitude;

    m_fix.track = p->fix.track;
    m_fix.speed = p->fix.speed;

    m_fix.pdop = p->dop.pdop;
    m_fix.hdop = p->dop.hdop;
    m_fix.vdop = p->dop.vdop;
    m_fix.tdop = p->dop.tdop;
    m_fix.gdop = p->dop.gdop;

    m_fix.err = p->fix.eph;
    m_fix.err_vert = p->fix.epv;
    m_fix.err_track = p->fix.epd;
    m_fix.err_speed = p->fix.eps;
    m_fix.err_time = p->fix.ept;
  }

  if(p->fix.mode >= MODE_2D){
    gps_fix_pub.publish(m_fix);
    m_last_was_no_fix = false;
  }

  if(p->fix.mode == MODE_NO_FIX){
    if(!m_last_was_no_fix)
      gps_fix_pub.publish(m_fix);
    m_last_was_no_fix = true;
  }
}
