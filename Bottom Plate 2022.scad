$fn = 30;

w2 = 29;
w1 = 128;

qtr_width = 62;
color_width = 22;
bosch_width = 25;
bosch_length = 20;
I2C_width = 27;
I2C_length = 38;
MUX_side = 32;

module roundedcube(size = [1, 1, 1], center = false, radius = 0.5, apply_to = "all") {
    // If single value, convert to [x, y, z] vector
    size = (size[0] == undef) ? [size, size, size] : size;

    translate_min = radius;
    translate_xmax = size[0] - radius;
    translate_ymax = size[1] - radius;
    translate_zmax = size[2] - radius;

    diameter = radius * 2;

    module build_point(type = "sphere", rotate = [0, 0, 0]) {
        if (type == "sphere") {
            sphere(r = radius);
        } else if (type == "cylinder") {
            rotate(a = rotate)
            cylinder(h = diameter, r = radius, center = true);
        }
    }

    obj_translate = (center == false) ?
        [0, 0, 0] : [
            -(size[0] / 2),
            -(size[1] / 2),
            -(size[2] / 2)
        ];

    translate(v = obj_translate) {
        hull() {
            for (translate_x = [translate_min, translate_xmax]) {
                x_at = (translate_x == translate_min) ? "min" : "max";
                for (translate_y = [translate_min, translate_ymax]) {
                    y_at = (translate_y == translate_min) ? "min" : "max";
                    for (translate_z = [translate_min, translate_zmax]) {
                        z_at = (translate_z == translate_min) ? "min" : "max";

                        translate(v = [translate_x, translate_y, translate_z])
                        if (
                            (apply_to == "all") ||
                            (apply_to == "xmin" && x_at == "min") || (apply_to == "xmax" && x_at == "max") ||
                            (apply_to == "ymin" && y_at == "min") || (apply_to == "ymax" && y_at == "max") ||
                            (apply_to == "zmin" && z_at == "min") || (apply_to == "zmax" && z_at == "max")
                        ) {
                            build_point("sphere");
                        } else {
                            rotate = 
                                (apply_to == "xmin" || apply_to == "xmax" || apply_to == "x") ? [0, 90, 0] : (
                                (apply_to == "ymin" || apply_to == "ymax" || apply_to == "y") ? [90, 90, 0] :
                                [0, 0, 0]
                            );
                            build_point("cylinder", rotate);
                        }
                    }
                }
            }
        }
    }
}


module Hangar_Tof(x,y,z){
    
    translate([x,y,z])difference(){
    
    //roundedcube([w2, 2, 23], false, 2, "y");
    cube([w2,3,23]);
    
    translate([5.5,4,20])rotate(90,[1,0,0])
        cylinder(5,2,2);
    
    translate([w2-5.5,4,20])rotate(90,[1,0,0])
        cylinder(5,2,2);
    
    for(i = [2:(13-2)/2:13]){
        for(n = [4:(w2-8)/2:w2-4]){
           translate([n,4,i]) rotate(90,[1,0,0])
                cylinder(5,1.5,1.5);
        }
    }
    
  
    }
    
}


module Qtr(x,y,z)
     translate([x,y,z]){
         translate([0,0,-1])cube([qtr_width,22,3]);
         
         translate([18,18,-1])cube([28,2,6]);
         translate([23,16,-1])cube([21,2,6]);
         
         for(i = [3:qtr_width - 6:qtr_width - 3]){
         translate([i,3,-1]) cylinder(6,1.5,1.5);
         }
     }
     
module Color(x,y,z){
    translate([x,y,z]){
    translate([0,0,-1])cube([color_width, color_width, 3]);
    translate([2,18,-1])cube([18,2,6]);
        for(i = [3:color_width-6:color_width - 3]){
            translate([i,3,0]) cylinder(6,1.5,1.5);
        }   
    }
}

module Bosch(x,y,z){
    translate([x,y,z]){
        translate([-3,0,-1])cube([bosch_width+3,bosch_length,3]);
        translate([-3,bosch_length/2-6/2,-1]) cube([2,6,6]);
        
        for(b = [2:bosch_length - 4: bosch_length - 2]){
            for(a = [2:bosch_width - 4:bosch_width - 2]){
               translate([a,b,-1]) cylinder(6,1.5,1.5);
            }
        }
        
    }
}

module I2C(x,y,z){
     translate([x,y,z-1]){         
         cube([I2C_width,I2C_length,3]);
         for(n = [3.5:I2C_width - 7:I2C_width - 3.5])
             for(i = [3.5:I2C_length - 7:I2C_length-3.5])
                translate([n,i,0]) cylinder(6,1.5,1.5);
         translate([I2C_width/2-13/2,4,0])cube([13,35,6]);
         
     }
    
}
module MUX(x,y,z){
    translate([x,y,z-1]){
        cube([MUX_side, MUX_side+1,3]);
        translate([MUX_side/2 - 11,22,0]) cube([22,11,6]);
        translate([MUX_side/2 - MUX_side/2,9,0]) cube([MUX_side,13,6]);
        translate([MUX_side/2 - 6.5,-2,0]) cube([13,11,6]);
        for(a = [3.5:MUX_side-7:MUX_side-3.5]){
            translate([a,3.5,0]) cylinder(6,1.5,1.5);
        }
        
    }
}





//I2C(w1-27,72-40,0);
    
translate([0,0,-1]){
    
    Hangar_Tof(w1/2 - w2/2,0,4);

difference(){
    
    
    roundedcube([w1, 75, 4], false, 2, "z");
    //cube([96,72,4]);
    //for(a = [24:w1-48:w1-24])
        
    for(a = [8:16:w1-8]){
        if(a != 56 && a != 72){
            translate([a,68,-1]) cylinder(6,2,2);
        }
    }
    Qtr(w1/2- qtr_width/2,30,0);

    for(i = [w1/2 - color_width-10:42:w1/2 + color_width -10])
        Color(i,5,0);
    Bosch(w1/2 - bosch_width/2,52,0);
    I2C(w1/2-60,25,0); 
    MUX(w1/2+64-MUX_side,30,0);
    
}




}



