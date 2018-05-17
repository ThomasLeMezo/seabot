#include "neom8l.h"

NeoM8L::NeoM8L(){
  // Parser init
  nmea_zero_INFO(&m_info);
  nmea_parser_init(&m_parser);

  // Projections

  if (!(pj_lambert = pj_init_plus("+init=epsg:2154"))){
      ROS_WARN("[GNSS NEOM8L] Error Lambert \n");
      exit(1);
    }

    if (!(pj_latlong = pj_init_plus("+init=epsg:4326")))    {
      ROS_WARN("[GNSS NEOM8L] Error LatLong \n");
      exit(1);
  }

}

NeoM8L::~NeoM8L(){
  close(m_file);
}


int NeoM8L::i2c_open(){
  if ((m_file = open(m_i2c_periph,O_RDWR)) < 0) {
    ROS_WARN("[GNSS NEOM8L] Failed to open the I2C bus");
    exit(1);
  }

  if (ioctl(m_file,I2C_SLAVE,m_i2c_addr) < 0) {
    ROS_WARN("[GNSS NEOM8L] Failed to acquire bus access and/or talk to slave");
    exit(1);
  }
  return 0;
}

int NeoM8L::read_data(){
  // Number of byte available
  unsigned char buff_nb_byte[2];
//   i2c_smbus_read_i2c_block_data : issue with kernel < 2.6.23 (?)
//   i2c_smbus_read_block_data is limited to 32bit (SMBUS protocol), => use the "i2c" version
  i2c_smbus_read_i2c_block_data(m_file, 0xFD, 2, buff_nb_byte); // OxFD, 0xFE (16 bit)
  unsigned int nb_byte = (buff_nb_byte[0] << 8) | buff_nb_byte[1] << 0; // test if i2c_smbus_write_word_data works ?

//  if(nb_byte>32)
    nb_byte = 32;

  uint8_t buff[nb_byte];
  i2c_smbus_read_i2c_block_data(m_file, 0xFF, nb_byte,buff);

  for(size_t i=0; i<nb_byte; i++){
    if(buff[i]!=0xFF){
      m_string_buff << buff[i];
      if(buff[i] == 0x0A){ // 0x0A = \n ? ou 0x0D ?
        std::string line = m_string_buff.str();
        nmea_parse(&m_parser, line.c_str(), line.length(), &m_info);
        m_string_buff.str(std::string()); // Clear the string buffer
      }
    }
    else
      break;
  }
}

double degMinSec2Deg(const double & val){
  double degree = trunc(val/1e2);
  double min = trunc((val/1e2-degree)*1e2);
  double sec = val - trunc(val);
  return degree + (min+sec/60.0)/60.0;
}

void NeoM8L::convert_data(){
  // [degree][min].[sec/60]
  m_lat = degMinSec2Deg(m_info.lat);
  m_lon = degMinSec2Deg(m_info.lon);

  // Warning : x <-> lon and y <-> lat
  double x = m_lon; // Longitude
  double y = m_lat; // Latitude
  pj_transform(pj_latlong, pj_lambert, 1, 1, &x, &y, NULL);
  m_east = x;
  m_north = y;

  m_time_month = m_info.utc.sec + (m_info.utc.min+ (m_info.utc.hour + m_info.utc.day*24)*60)*60;
}
