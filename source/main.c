#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <libxml/parser.h>
#include "constants.h"
#include "asv.h"
#include "wave.h"
#include "wind.h"
#include "current.h"

int main(int argc, char** argv)
{
  // Parse the input file.
  char* filename = argv[1];
  if(argc < 2)
  {
    fprintf(stderr, "Error. Usage: %s config_file.xml.\n", argv[0]);
    return 1;
  }
  xmlDoc* document = xmlReadFile(filename, NULL, 256); // 256:remove blank nodes
  if(!document)
  {
    fprintf(stderr, "Error. Cannot find file %s.", filename);
    return 1;
  }
  xmlNode* root = document->children;
  
  // Environment specifications.
  
  // Wind data
  xmlNode* node = root->children; // This should be wind
  if(strcmp(node->name, "wind"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \ 
                     Expected node wind but found %s. \n", 
            node->name);
    return 1;
  }
  fprintf(stdout, "WIND: \n");
  // Wind speed
  double wind_speed = 0.0;
  bool is_wind_speed_available = false;
  xmlNode* sub_node = node->children; // This should be speed
  if(strcmp(sub_node->name, "speed"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \ 
                    Expected node speed but found %s. \n", 
            sub_node->name);
    return 1;
  }
  xmlNode* val_node = sub_node->children;
  if(val_node != NULL)
  {
    wind_speed = atof(val_node->content);
    is_wind_speed_available = true;
    fprintf(stdout, "--> speed = %f m/s.\n", wind_speed);
  }
  else
  {
    fprintf(stdout, "--> speed = n/a.\n");
  }
  // Wind direction
  double wind_direction = 0.0;
  bool is_wind_direction_available = false;
  sub_node = sub_node->next; // This should be direction
  if(strcmp(sub_node->name, "direction"))
  {
    fprintf(stderr,
            "Error. Incorrect xml schema. \
            Expected node direction but found %s. \n",
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    wind_direction = atof(val_node->content);
    is_wind_direction_available = true;
    fprintf(stdout, "--> direction = %f radians.\n", wind_direction);
  }
  else
  {
    fprintf(stdout, "--> direction = n/a.\n");
  }
 
  // Current data
  node = node->next; // This should be current
  if(strcmp(node->name, "current"))
  {
    fprintf(stderr, 
            "Error. Incorrect xml schema. \
            Expected node current but found %s. \n", 
            node->name);
    return 1;
  }
  fprintf(stdout, "CURRENT: \n");
  // Current speed
  double current_speed = 0.0;
  bool is_current_speed_available = false;
  sub_node = node->children; // This should be speed
  if(strcmp(sub_node->name, "speed"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node speed but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    current_speed = atof(val_node->content);
    is_current_speed_available = true;
    fprintf(stdout, "--> speed = %f m/s.\n", current_speed);
  }
  else
  {
    fprintf(stdout, "--> speed = n/a.\n");
  }
  // Current direction
  double current_direction = 0.0;
  bool is_current_direction_available = false;
  sub_node = sub_node->next; // This should be direction
  if(strcmp(sub_node->name, "direction"))
  {
    fprintf(stderr,
            "Error. Incorrect xml schema. \ 
            Expected node direction but found %s. \n",
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    current_direction = atof(val_node->content);
    is_current_direction_available = true;
    fprintf(stdout, "--> direction = %f radians.\n", current_direction);
  }
  else
  {
    fprintf(stdout, "--> direction = n/a.\n");
  }
  
  // Wave data
  node = node->next; // This should be wave
  if(strcmp(node->name, "wave"))
  {
    fprintf(stderr, 
            "Incorrect xml schema. Expected node wave but found %s. \n", 
            node->name);
    return 1;
  }
  fprintf(stdout, "WAVE: \n");
  // Wave spectrum based on wind
  bool is_wave_spectrum_based_on_wind = false;
  sub_node = node->children; // This should be wind
  if(strcmp(sub_node->name, "wind"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \ 
                    Expected node wind but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    if(strcmp(val_node->content, "true"))
    {
      is_wave_spectrum_based_on_wind = true;
    }
    else if(strcmp(val_node->content, "false"))
    {
      is_wave_spectrum_based_on_wind = false;
    } 
    else
    {
      fprintf(stderr,
          "Error. Incorrect xml schema. \
           Expected value of node - true or false, but found %s. \n", 
           val_node->content);
    }
    fprintf(stdout, "--> is wave data based on wind = %i.\n", 
            is_wave_spectrum_based_on_wind);

  }
  else
  {
    fprintf(stdout, "--> wind data = n/a.\n");
  }
  // Wave spectrum based on significant wave height.
  double sig_wave_height = 0.0;
  bool is_sig_wave_ht_available = false;
  sub_node = sub_node->next; // This should be sig_wave_ht
  if(strcmp(sub_node->name, "sig_wave_ht"))
  {
    fprintf(stderr,
            "Error. Incorrect xml schema. \ 
            Expected node sig_wave_ht but found %s. \n",
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    sig_wave_height = atof(val_node->content);
    is_sig_wave_ht_available = true;
    fprintf(stdout, "--> significant wave height = %f m.\n", sig_wave_height);
  }
  else
  {
    fprintf(stdout, "--> significant wave height = n/a.\n");
  }
  // Wave spectrum based on peak spectral frequency.
  double peak_spectral_freq = 0.0;
  bool is_peak_spectral_freq_available = false;
  sub_node = sub_node->next; // This should be peak_spectral_freq
  if(strcmp(sub_node->name, "peak_spectral_freq"))
  {
    fprintf(stderr,
            "Error. Incorrect xml schema. \
            Expected node peak_spectral_freq but found %s. \n",
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    peak_spectral_freq = atof(val_node->content);
    is_peak_spectral_freq_available = true;
    fprintf(stdout, "--> peak spectral frequency = %f Hz.\n", peak_spectral_freq);
  }
  else
  {
    fprintf(stdout, "--> peak spectral frequency = n/a.\n");
  }

  // Create objects for environment model
  fprintf(stdout, "ENVIRONMENT MODEL:\n");
  struct Wind wind;
  struct Current current;
  struct Wave wave;
  // Also create pointers to these models
  struct Wind* p_wind = NULL;
  struct Current* p_current = NULL;
  struct Wave* p_wave = NULL;
  // Initialise the models based on input data
  if(is_wind_speed_available && is_wind_direction_available)
  {
    wind_init(&wind, wind_speed, wind_direction);
    p_wind = &wind;
    fprintf(stdout, "--> wind model created.\n");
  }
  else
  {
    fprintf(stdout, "--> wind model = NULL.\n");
  }
  if(is_current_speed_available && is_current_direction_available)
  {
    current_init(&current, current_speed, current_direction);
    p_current = &current;
    fprintf(stdout, "--> current model created.\n");
  }
  else
  {
    fprintf(stdout, "--> current model = NULL.\n");
  }
  if(is_wave_spectrum_based_on_wind)
  {
    if(!p_wind)
    {
      fprintf(stderr, "Error. Missing data. \ 
                       Wave model is based on wind but found no wind data. \n");
      return 1;
    }
    wave_init_with_wind(&wave, p_wind);
    p_wave = &wave;
    fprintf(stdout, "--> wave model created based on wind model.\n");
  }
  else if(is_sig_wave_ht_available)
  {
    wave_init_with_sig_wave_ht(&wave, sig_wave_height);
    p_wave = &wave;
    fprintf(stdout, "--> wave model created based on \
                    significant wave height.\n");
  }
  else if(is_peak_spectral_freq_available)
  {
    wave_init_with_peak_freq(&wave, peak_spectral_freq);
    p_wave = &wave;
    fprintf(stdout, "--> wave model created based on \
                    peak spectral frequency.\n");
  }
  else
  {
    fprintf(stdout, "--> wave model = NULL.\n");
  }

 
  // ASV data
  node = node->next; // This should be asv_spec
  if(strcmp(node->name, "asv_spec"))
  {
    fprintf(stderr, 
            "Error. Incorrect xml schema. \
            Expected node asv_spec but found %s. \n", 
            node->name);
    return 1;
  }
  fprintf(stdout, "ASV SPECIFICATION: \n");
  // L_wl
  double L_wl = 0.0;
  sub_node = node->children; // This should be L_wl
  if(strcmp(sub_node->name, "L_wl"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node L_wl but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    L_wl = atof(xmlNodeGetContent(val_node));
    fprintf(stdout, "--> L_wl = %f m.\n", L_wl);
  }
  else
  {
    fprintf(stderr, "Error. Missing data L_wl.\n");
    return 1;
  }
  // B_wl
  double B_wl = 0.0;
  sub_node = sub_node->next; // This should be B_wl
  if(strcmp(sub_node->name, "B_wl"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node B_wl but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    B_wl = atof(val_node->content);
    fprintf(stdout, "--> B_wl = %f m.\n", B_wl);
  }
  else
  {
    fprintf(stderr, "Error. Missing data B_wl.\n");
    return 1;
  }
  // D
  double D = 0.0;
  sub_node = sub_node->next; // This should be D
  if(strcmp(sub_node->name, "D"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node D but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    D = atof(val_node->content);
    fprintf(stdout, "--> D = %f m.\n", D);
  } 
  else
  {
    fprintf(stderr, "Error. Missing data D.\n");
    return 1;
  }
  // T
  double T = 0.0;
  sub_node = sub_node->next; // This should be T
  if(strcmp(sub_node->name, "T"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node T but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    T = atof(val_node->content);
    fprintf(stdout, "--> T = %f m.\n", T);
  }
  else
  {
    fprintf(stderr, "Error. Missing data T. \n");
    return 1;
  }
  // max_speed 
  double max_speed = 0.0;
  sub_node = sub_node->next; // This should be max_speed
  if(strcmp(sub_node->name, "max_speed"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node max_speed but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    max_speed = atof(val_node->content);
    fprintf(stdout, "--> max_speed = %f m/s.\n", max_speed);
  }
  else
  {
    fprintf(stderr, "Error. Missing data max_speed. \n");
    return 1;
  }
  // KG 
  double KG = 0.0;
  sub_node = sub_node->next; // This should be KG
  if(strcmp(sub_node->name, "KG"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node KG but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    KG = atof(val_node->content);
    fprintf(stdout, "--> KG = %f m.\n", KG);
  }
  else
  {
    fprintf(stderr, "Error. Missing data KG. \n");
    return 1;
  }
  // disp 
  double disp = 0.0;
  sub_node = sub_node->next; // This should be disp
  if(strcmp(sub_node->name, "disp"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node disp but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    disp = atof(val_node->content);
    fprintf(stdout, "--> disp = %f m3.\n", disp);
  }
  else
  {
    fprintf(stderr, "Error. Missing data disp. \n");
    return 1;
  }
  // r_roll 
  double r_roll = 0.0;
  sub_node = sub_node->next; // This should be r_roll
  if(strcmp(sub_node->name, "r_roll"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node r_roll but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    r_roll = atof(val_node->content);
    fprintf(stdout, "--> r_roll = %f m.\n", r_roll);
  }
  else
  {
    fprintf(stderr, "Error. Missing data r_roll. \n");
    return 1;
  }
  // r_pitch
  double r_pitch = 0.0;
  sub_node = sub_node->next; // This should be r_pitch
  if(strcmp(sub_node->name, "r_pitch"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node r_pitch but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    r_pitch = atof(val_node->content);
    fprintf(stdout, "--> r_pitch = %f m.\n", r_pitch);
  }
  else
  {
    fprintf(stderr, "Error. Missing data r_pitch. \n");
    return 1;
  }
  // r_yaw
  double r_yaw = 0.0;
  sub_node = sub_node->next; // This should be r_yaw
  if(strcmp(sub_node->name, "r_yaw"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node r_yaw but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    r_yaw = atof(val_node->content);
    fprintf(stdout, "--> r_yaw = %f m.\n", r_yaw);
  }
  else
  {
    fprintf(stderr, "Error. Missing data r_yaw. \n");
    return 1;
  }

  // Create object for ASV specification
  struct Asv_specification asv_spec;
  asv_spec.L_wl = L_wl;
  asv_spec.B_wl = B_wl; 
  asv_spec.D = D;
  asv_spec.T = T;
  asv_spec.max_speed = max_speed;
  asv_spec.KG = KG;
  asv_spec.disp = disp;
  asv_spec.r_roll = r_roll;
  asv_spec.r_pitch = r_pitch;
  asv_spec.r_yaw = r_yaw;
  // Create ASV model
  fprintf(stdout, "ASV MODEL:\n");
  struct Asv asv;
  asv_init(&asv, &asv_spec, p_wave, p_wind, p_current);
  fprintf(stdout, "--> asv model created with asv specification and \
environment model.\n");

  // ASV position
  node = node->next; // This should be asv_position
  if(strcmp(node->name, "asv_position"))
  {
    fprintf(stderr, 
            "Error. Incorrect xml schema. \
            Expected node asv_position but found %s. \n", 
            node->name);
    return 1;
  }
  fprintf(stdout, "ASV POSITION: \n");
  // x
  double x = 0.0;
  bool is_x_available = false;
  sub_node = node->children; // This should be x
  if(strcmp(sub_node->name, "x"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node x but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    x = atof(xmlNodeGetContent(val_node));
    is_x_available = true;
    fprintf(stdout, "--> x = %f m.\n", x);
  }
  else
  {
    fprintf(stdout, "--> x = n/a.\n");
  }
  // y
  double y = 0.0;
  bool is_y_available = false;
  sub_node = sub_node->next; // This should be y
  if(strcmp(sub_node->name, "y"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node y but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    y = atof(xmlNodeGetContent(val_node));
    is_y_available = true;
    fprintf(stdout, "--> y = %f m.\n", y);
  }
  else
  {
    fprintf(stdout, "--> y = n/a.\n");
  }
  // z
  double z = 0.0;
  bool is_z_available = false;
  sub_node = sub_node->next; // This should be z
  if(strcmp(sub_node->name, "z"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node z but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    z = atof(xmlNodeGetContent(val_node));
    is_z_available = true;
    fprintf(stdout, "--> z = %f m.\n", z);
  }
  else
  {
    fprintf(stdout, "--> z = n/a.\n");
  }
  if(is_x_available && is_y_available && is_z_available)
  {
    struct Point position = (struct Point){x,y,z};
    asv_set_position(&asv,position);
    fprintf(stdout, "--> position set to (%f, %f, %f).\n", x,y,z);
  }
  else
  {
    fprintf(stdout, "--> position set to (0.0, 0.0, %f).\n", -T);
  }
  
  // ASV attitude
  node = node->next; // This should be asv_attitude
  if(strcmp(node->name, "asv_attitude"))
  {
    fprintf(stderr, 
            "Error. Incorrect xml schema. \
            Expected node asv_attitude but found %s. \n", 
            node->name);
    return 1;
  }
  fprintf(stdout, "ASV ATTITUDE: \n");
  // heel
  double heel = 0.0;
  bool is_heel_available = false;
  sub_node = node->children; // This should be heel
  if(strcmp(sub_node->name, "heel"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node heel but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    heel = atof(xmlNodeGetContent(val_node));
    is_heel_available = true;
    fprintf(stdout, "--> heel = %f radians.\n", heel);
  }
  else
  {
    fprintf(stdout, "--> heel = n/a.\n");
  }
  // trim
  double trim = 0.0;
  bool is_trim_available = false;
  sub_node = sub_node->next; // This should be trim
  if(strcmp(sub_node->name, "trim"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node trim but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    trim = atof(xmlNodeGetContent(val_node));
    is_trim_available = true;
    fprintf(stdout, "--> trim = %f radians.\n", trim);
  }
  else
  {
    fprintf(stdout, "--> trim = n/a.\n");
  }
  // heading
  double heading = 0.0;
  bool is_heading_available = false;
  sub_node = sub_node->next; // This should be heading
  if(strcmp(sub_node->name, "heading"))
  {
    fprintf(stderr, "Error. Incorrect xml schema. \
                     Expected node heading but found %s. \n", 
            sub_node->name);
    return 1;
  }
  val_node = sub_node->children;
  if(val_node != NULL)
  {
    heading = atof(xmlNodeGetContent(val_node));
    is_heading_available = true;
    fprintf(stdout, "--> heading = %f radians.\n", heading);
  }
  else
  {
    fprintf(stdout, "--> heading = n/a.\n");
  }
  if(is_heel_available && is_trim_available && is_heading_available)
  {
    struct Asv_attitude attitude = (struct Asv_attitude){heel, trim, heading};
    asv_set_attitude(&asv, attitude);
    fprintf(stdout, "--> attitude set to (%f, %f, %f).\n", heel,trim,heading);
  }
  else
  {
    fprintf(stdout, "--> attitude set to (0.0, 0.0, 0.0).\n");
  }
 
  // Start simulation
  fprintf(stdout, "START SIMULATION: \n");
  double delta_t = 20.0/1000.0; // 20 milli-seconds
  for(double t = 0.0; t < 10.0; t += delta_t)
  {
    asv_set_dynamics(&asv, t);
    printf("\n %f , %f", 
        t, 
        asv.attitude.trim); 
  }
  return 0;
}
