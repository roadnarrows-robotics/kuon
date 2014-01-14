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

  this.draw = function()
  { 
    d3.xml("../img/xbox.svg", "image/svg+xml", function(xml) {
      document.getElementById('xbox_vis').appendChild(xml.documentElement);
      createButtons(XboxButtonInfoTable);
    });
  }

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
