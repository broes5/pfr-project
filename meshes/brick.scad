include </home/benjamin/3D Printing/BOSL2/std.scad>

$fa=1;
$fs=0.4;

// Brick dimensions (cm).
bheight=7.6;
bwidth=11;
blength=23;
wallt=4;	// Wall thickness.

module stub1() {
	cube([1.5,1.5,1.5]);
	translate([2.25,0.75,0])
		prismoid(size1=[1.5,1.5], size2=[0,1.5], shift=[-0.75,0], h=1.5);
	translate([0.75,2.25,0])
		prismoid(size1=[1.5,1.5], size2=[0,1.5], shift=[-0.75,0], h=1.5, spin=90);
}

module stub2() {
	cube([1.5,1.5,1.5]);
	translate([2.25,0.75,0])
		prismoid(size1=[1.5,1.5], size2=[0,1.5], shift=[-0.75,0], h=1.5);
}

module brick() {
	difference() {
		cube([blength, bwidth, bheight], true);
		translate([0,0,-bheight/2])
			linear_extrude(bheight)
				rect([blength-wallt,bwidth-wallt], rounding=2);
		translate([-blength/2,-bwidth/2,-bheight/2])
			stub1();
		translate([blength/2,bwidth/2,-bheight/2])
			rotate([0,0,180])
				stub1();
		translate([-blength/2,bwidth/2,-bheight/2])
			rotate([0,0,-90])
				stub1();
		translate([blength/2,-bwidth/2,-bheight/2])
			rotate([0,0,90])
				stub1();
		translate([0,-bwidth/2,-bheight/2])
			stub1();
		translate([0,bwidth/2,-bheight/2])
			rotate([0,0,-90])
				stub1();
		translate([0,bwidth/2,-bheight/2])
			rotate([0,0,180])
				stub1();
		translate([0,-bwidth/2,-bheight/2])
			rotate([0,0,90])
				stub1();
	}
	translate([-blength/2,-bwidth/2,bheight/2])
		stub1();
	translate([blength/2,bwidth/2,bheight/2])
		rotate([0,0,180])
			stub1();
	translate([-blength/2,bwidth/2,bheight/2])
		rotate([0,0,-90])
			stub1();
	translate([blength/2,-bwidth/2,bheight/2])
		rotate([0,0,90])
			stub1();
	translate([0,-bwidth/2,bheight/2])
		stub2();
	translate([0,bwidth/2-1.5,bheight/2])
			stub2();
	translate([0,bwidth/2,bheight/2])
		rotate([0,0,180])
			stub2();
	translate([0,-bwidth/2+1.5,bheight/2])
		rotate([0,0,180])
			stub2();
}

module half_brick() {
	difference() {
		cube([blength/2, bwidth, bheight], true);
		translate([0,0,-bheight/2])
			linear_extrude(bheight)
				rect([blength/2-wallt,bwidth-wallt], rounding=2);
		translate([-blength/4,-bwidth/2,-bheight/2])
			stub1();
		translate([blength/4,bwidth/2,-bheight/2])
			rotate([0,0,180])
				stub1();
		translate([-blength/4,bwidth/2,-bheight/2])
			rotate([0,0,-90])
				stub1();
		translate([blength/4,-bwidth/2,-bheight/2])
			rotate([0,0,90])
				stub1();
	}
	translate([-blength/4,-bwidth/2,bheight/2])
		stub1();
	translate([blength/4,bwidth/2,bheight/2])
		rotate([0,0,180])
			stub1();
	translate([-blength/4,bwidth/2,bheight/2])
		rotate([0,0,-90])
			stub1();
	translate([blength/4,-bwidth/2,bheight/2])
		rotate([0,0,90])
			stub1();
}

half_brick();
//rotate([0,0,90])
//	translate([bwidth/1.9,blength/4,-bheight-2])
//		brick();
//translate([blength/2,0,-bheight-2])
//	brick();
