    /*
    * Ultrasonic Sensor HC-SR04 and Arduino Tutorial
    *
    * Crated by Dejan Nedelkovski,
    * www.HowToMechatronics.com
    *
    */
    #include <ros.h>
    #include <ros/time.h>
    #include <sensor_msgs/Range.h>
    #include <std_msgs/Header.h>
    #include <tf/transform_broadcaster.h>

    //Set up the ros node and publisher
    std_msgs::Header header;
    sensor_msgs::Range sonar_msg;
    ros::Publisher pub_sonar("sonar", &sonar_msg);
    ros::NodeHandle nh;

    geometry_msgs::TransformStamped t;
    tf::TransformBroadcaster broadcaster;

    char base_link[] = "/base_link";
    char sonar[] = "/sonar";
    
    // defines pins numbers
    const int trigPin = 12;
    const int echoPin = 11;
    // defines variables
    long duration;
    float distance;
    long publisher_timer;
    
    void setup() {
      pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
      pinMode(echoPin, INPUT); // Sets the echoPin as an Input
      Serial.begin(9600); // Starts the serial communication
      nh.initNode();
      nh.advertise(pub_sonar);
      broadcaster.init(nh);
    }

    
    
    void loop() {
      if (millis() > publisher_timer) {
         // Clears the trigPin
         digitalWrite(trigPin, LOW);
         delayMicroseconds(2);
         // Sets the trigPin on HIGH state for 10 micro seconds
         digitalWrite(trigPin, HIGH);
         delayMicroseconds(10);
         digitalWrite(trigPin, LOW);
         // Reads the echoPin, returns the sound wave travel time in microseconds
         duration = pulseIn(echoPin, HIGH);
         // Calculating the distance
         distance= duration*0.034/2;

         t.header.frame_id = base_link;
         t.child_frame_id = sonar;
         t.transform.translation.x = 1.0; 
         t.transform.rotation.x = 0.0;
         t.transform.rotation.y = 0.0; 
         t.transform.rotation.z = 0.0; 
         t.transform.rotation.w = 1.0;  
         t.header.stamp = nh.now();
         broadcaster.sendTransform(t);

         header.stamp = nh.now();
         header.frame_id = "/sonar";
         sonar_msg.header = header;
         sonar_msg.radiation_type = sonar_msg.ULTRASOUND;
         sonar_msg.field_of_view = 1;
         sonar_msg.min_range = 0.02;
         sonar_msg.max_range = 4;
         sonar_msg.range = distance/100;
         
         pub_sonar.publish(&sonar_msg);

         publisher_timer = millis() + 500; //publish once a second
        
      }

      nh.spinOnce();
      
      // Prints the distance on the Serial Monitor
  //    Serial.print("Distance: ");
  //    Serial.println(distance);
    }
