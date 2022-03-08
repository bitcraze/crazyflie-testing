width = 55;
depth = 20;
height = 2;
hookDist = width - 6;
hole = 2;

module hook() {
    hookWidth = 6;
    hookDepth = 4;
    hookHeight = 18;
    overHang = 4;
    overHangHeigt = 4;
    
    translate([-hookWidth / 2, - hookDepth / 2, 0]) {
        cube([hookWidth, hookDepth, hookHeight]);
    }
    translate([-hookWidth / 2, -hookDepth / 2, hookHeight]) {
        cube([hookWidth, hookDepth + overHang, overHangHeigt]);
    }
}


difference() {
    union(){    
        translate([-width / 2, 0, 0]) {
            cube([width, depth, height]);
        }

        translate([-hookDist / 2, 3, 0]) {
            rotate([0, 0, -45]) {
                hook();
            }
        }

        translate([hookDist / 2, 3, 0]) {
            rotate([0, 0, 45]) {
                hook();
            }
        }
    }
    
    translate([10, 12, -1]) cylinder(height + 2, hole, hole);
    translate([-10, 12, -1]) cylinder(height + 2, hole, hole);
}