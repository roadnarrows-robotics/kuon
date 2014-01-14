var XboxButtonInfoTable = {
  "a_button": {
    "id"       : "a_button",
    "center_x" : 730,
    "center_y" : 270,
    "radius"   : 30,
    "stroke"   : "#060",
    "fill"     : "#282"},

  "b_button": {
    "id"       : "b_button",  
    "center_x" : 795,
    "center_y" : 205,
    "radius"   : 30,
    "stroke"   : "#800",
    "fill"     : "#a00"},

  "x_button": {
    "id"       : "x_button",
    "center_x" : 665,
    "center_y" : 205,
    "radius"   : 30,
    "stroke"   : "#04d",
    "fill"     : "#06f"},

  "y_button": {
    "id"       : "y_button",
    "center_x" : 730,
    "center_y" : 140,
    "radius"   : 30,
    "stroke"   : "#f90",
    "fill"     : "#fb0"},
  
  "center_button": {
    "id"       : "center_button",
    "center_x" : 490,
    "center_y" : 200,
    "radius"   : 50,
    "stroke"   : "#bbb",
    "fill"     : "#999"},
  
  "back_button": {
    "id"       : "back_button",
    "center_x" : 400,
    "center_y" : 200,
    "radius"   : 15,
    "stroke"   : "#222",
    "fill"     : "#444"},

  "start_button": {
    "id"       : "start_button",
    "center_x" : 580,
    "center_y" : 200,
    "radius"   : 15,
    "stroke"   : "#222",
    "fill"     : "#444"},

  "left_joy_shroud"   : {
    "center_x" : 240,
    "center_y" : 190,
    "radius"   : 75,
    "stroke"   : "#999",
    "fill"     : "#999"},

  "left_joy"   : {
    "id"       : "left_joy",
    "center_x" : 240,
    "center_y" : 190,
    "radius"   : 60,
    "stroke"   : "#222",
    "fill"     : "#444"},

  "right_joy_shroud" : {
    "center_x" : 620,
    "center_y" : 370,
    "radius"   : 75,
    "stroke"   : "#999",
    "fill"     : "#999"},

  "right_joy"   : {
    "id"       : "right_joy",
    "center_x" : 620,
    "center_y" : 370,
    "radius"   : 60,
    "stroke"   : "#222",
    "fill"     : "#444"},

  "dpad_shroud"   : {
    "center_x" : 360,
    "center_y" : 370,
    "radius"   : 75,
    "stroke"   : "#999",
    "fill"     : "#999"},

  "dpad_back"   : {
    "id"       : "dpad_back",
    "center_x" : 360,
    "center_y" : 370,
    "radius"   : 60,
    "stroke"   : "#222",
    "fill"     : "#444"}
};

function buttons_circle(id,center_x,center_y,radius,stroke,fill)
{                                    
  console.info("here: buttons_circle");
  var circle = d3.select("#buttons").append("circle")
                           .attr("id", id)
                           .attr("cx", center_x)
                           .attr("cy", center_y)
                           .attr("r", radius)
                           .style("stroke", stroke)
                           .style("fill", fill);  
}

function createButtons(buttonInfo)
{
  for (bttn_key in buttonInfo)
  {
    bttn=buttonInfo[bttn_key]

    var Id       = bttn.id;
    var Center_x = bttn.center_x;
    var Center_y = bttn.center_y;
    var Radius   = bttn.radius;
    var Stroke   = bttn.stroke;
    var Fill     = bttn.fill;
    
    buttons_circle(Id,Center_x,Center_y,Radius,Stroke,Fill)
  }
}
