xbox.sub_state(function(message){
  if (message.a_button==0)
  {
    d3.selectAll("#a_button").style("fill", "#080");
  }
  else
  {
    d3.selectAll("#a_button").style("fill", "#0a0");
  }

  if (message.b_button==0)
  {
    d3.selectAll("#b_button").style("fill", "#a00");
  }
  else
  {
    d3.selectAll("#b_button").style("fill", "#c00");
  }

  if (message.x_button==0)
  {
    d3.selectAll("#x_button").style("fill", "#06f");
  }
  else
  {
    d3.selectAll("#x_button").style("fill", "#09f");
  }

  if (message.y_button==0)
  {
    d3.selectAll("#y_button").style("fill", "#fb0");
  }
  else
  {
    d3.selectAll("#y_button").style("fill", "fc0");
  }

  if (message.center_button==0)
  {
    d3.selectAll("#center_button").style("fill", "#999")
    d3.selectAll(".X").style("fill", "#0a0");
  }
  else
  {
    d3.selectAll("#center_button").style("fill", "#777")
    d3.selectAll(".X").style("fill", "#0c0");
  }

  if (message.start_button==0)
  {
    d3.selectAll("#start_button").style("fill", "#444");
  }
  else
  {
    d3.selectAll("#start_button").style("fill", "#777");
  }

  if (message.back_button==0)
  {
    d3.selectAll("#back_button").style("fill", "#444");
  }
  else
  {
    d3.selectAll("#back_button").style("fill", "#777");
  }

  d3.selectAll("#right_joy").attr("cy",(-1*(message.right_joy_y/1000)+370).toString())
                         .attr("cx",((message.right_joy_x/1000)+620).toString());
  d3.selectAll("#left_joy").attr("cy",(-1*(message.left_joy_y/1000)+190).toString())
                        .attr("cx",((message.left_joy_x/1000)+240).toString());

  if(message.dpad_left > 0)
  {
    d3.selectAll("#dpad").attr("transform", "translate(-5,0)")
                      .style("fill","#555");
  }
  else if(message.dpad_right > 0)
  {
    d3.selectAll("#dpad").attr("transform", "translate(5,0)")
                      .style("fill","#555");
  }
  else if(message.dpad_up > 0)
  {
    d3.selectAll("#dpad").attr("transform", "translate(0,-5)")
                      .style("fill","#555");
  }
  else if(message.dpad_down > 0)
  {
    d3.selectAll("#dpad").attr("transform", "translate(0,5)")
                      .style("fill","#555");
  }
  else
  {
    d3.selectAll("#dpad").attr("transform","translate(0,0)")
                      .style("fill","#444");
  }  

  d3.selectAll("#left_trig").attr("transform","translate (0,"+(message.left_trig/10)+")");
  d3.selectAll("#right_trig").attr("transform","translate (0,"+(message.right_trig/10)+")");

  if (message.left_bump==0)
  {
    d3.selectAll("#left_bump").attr("transform","translate(0,0)");
  }
  else
  {
    d3.selectAll("#left_bump").attr("transform","translate(0,8)");
  }
  if (message.right_bump==0)
  {
    d3.selectAll("#right_bump").attr("transform","translate(0,0)");
  }
  else
  {
    d3.selectAll("#right_bump").attr("transform","translate(0,8)");
  }

  xbox.pub_rumble(message.left_trig, message.right_trig);
  
  if(message.a_button) {xbox.set_led(xbox.led_pattern.ALL_OFF);}
  else if(message.b_button) {xbox.set_led(xbox.led_pattern.ALL_SPIN);}
  else if(message.x_button) {xbox.set_led(xbox.led_pattern.ALL_SPIN2);}
  else if(message.y_button) {xbox.set_led(xbox.led_pattern.ALL_ON_BLINK);}
    }); 
