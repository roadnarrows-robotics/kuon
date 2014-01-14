// 
// xbox360 controller rosbridge handle
// 
// required js imports:
//    http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js
//    http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js
//    rosbridge_global.js
//

function xbox360(throttle_rate) {
  // set default throttle rate = 0
  this.throttle_rate = typeof throttle_rate !== 'undefined' ? throttle_rate : 0;

  /* 
   * @brief Subscribe to the xbox360 controller state.
   *
   * @param cb : Callback to process the return hid/Controller360State message
   **/
  this.sub_state = function(cb) {
    if(typeof cb == 'undefined') {
      console.error("When subscribing to a topic, you must provide a callback");
      return;
    }
    this.state_topic.subscribe(function(msg){cb(msg);});
  }

  /*
   * @brief Publish a command to the rumble topic
   *
   * @param left  : Left rumbler setting
   * @param right : Right rumbler setting
   * @param cb    : Optional callback to process the return code
   **/
  this.pub_rumble = function(left, right, cb) {
    // set default callback
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};

    var req = new ROSLIB.Message({
      left_rumble:left,
      right_rumble:right
    });

    this.rumble_topic.publish(req, function(rsp){cb(rsp)});
  }

  /*
   * @brief Publish a command to the rumble topic
   *
   * @param pattern  : Left rumbler setting
   * @param cb    : Optional callback to process the return code
   **/
  this.set_led = function(pattern, cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};

    var msg = new ROSLIB.Message({
      val:pattern,
    });

    var req = new ROSLIB.ServiceRequest({
      led_pattern:msg
    })

    this.set_led_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.ping = function(cb) {
    cb = typeof cb !== 'undefined' ? cb : function(rsp){};
    var req = new ROSLIB.ServiceRequest({});
    this.ping_srv.callService(req, function(rsp){cb(rsp);});
  }

  this.draw = function()
  { 
    d3.xml("../img/xbox.svg", "image/svg+xml", function(xml) {
      document.getElementById('xbox_vis').appendChild(xml.documentElement);
      createButtons(XboxButtonInfoTable);
    });
  }

  // Available LED patterns
  this.led_pattern = {
    ALL_OFF       : 0,
    ALL_ON_BLINK  : 1,
    LED1_BLINK_ON : 2,
    LED2_BLINK_ON : 3,
    LED3_BLINK_ON : 4,
    LED4_BLINK_ON : 5,
    LED1_ON       : 6,
    LED2_ON       : 7,
    LED3_ON       : 8,
    LED4_ON       : 9,
    ALL_SPIN      : 10,
    LED4_BLINK_LONG : 11,
    LED4_BLINK    : 12,
    ALL_SPIN2     : 13,
    ALL_BLINK     : 14
  };

  this.default_state = {
    a_button: 0,
    b_button: 0,
    back_button: 0,
    center_button: 0,
    dpad_down: 0,
    dpad_left: 0,
    dpad_right: 0,
    dpad_up: 0,
    left_bump: 0,
    left_joy_click: 0,
    left_joy_x: 0,
    left_joy_y: 0,
    left_trig: 0,
    right_bump: 0,
    right_joy_click: 0,
    right_joy_x: 0,
    right_joy_y: 0,
    right_trig: 0,
    start_button: 0,
    x_button: 0,
    y_button: 0
  };

  //------------------------------------------------------------------------//
  //                          PRIVATE IMPLEMENTATION                        //
  //------------------------------------------------------------------------//

  // Topics
  this.state_topic = new ROSLIB.Topic({
    ros: ros,
    name: "/xbox_360/controller_360_state",
    messageType: "hid/Controller360State",
    throttle_rate : this.throttle_rate
  });

  this.rumble_topic = new ROSLIB.Topic({
    ros: ros,
    name: "/xbox_360/rumble_command",
    messageType: "hid/RumbleCmd"
  })

  //Services
  this.ping_srv = new ROSLIB.Service({
    ros: ros,
    name: "/xbox_360/ping_controller",
    messageType: "hid/Ping"
  });

  this.set_led_srv = new ROSLIB.Service({
    ros: ros,
    name: "/xbox_360/set_led",
    messageType: "hid/SetLED"
  });


}

xbox = new xbox360(50);
