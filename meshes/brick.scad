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

brick();
translate([blength,0,0])
echo([blength,0,0])
	brick();
translate([blength*2,0,0])
echo([blength*2,0,0])
	brick();
translate([blength*3,0,0])
echo([blength*3,0,0])
	brick();
translate([blength*4,0,0])
echo([blength*4,0,0])
	brick();
// New layer.
translate([blength*0.5,0,bheight+1])
echo([blength*0.5,0,bheight+1])
	brick();
translate([blength*1.5,0,bheight+1])
echo([blength*1.5,0,bheight+1])
	brick();
translate([blength*2.5,0,bheight+1])
echo([blength*2.5,0,bheight+1])
	brick();
translate([blength*3.5,0,bheight+1])
echo([blength*3.5,0,bheight+1])
	brick();
translate([blength*4.5,0,bheight+1])
echo([blength*4.5,0,bheight+1])
	brick();
// New layer.
translate([0,0,bheight*2+2])
echo([0,0,bheight*2+2])
	brick();
translate([blength,0,bheight*2+2])
echo([blength,0,bheight*2+2])
	brick();
translate([blength*2,0,bheight*2+2])
echo([blength*2,0,bheight*2+2])
	brick();
translate([blength*3,0,bheight*2+2])
echo([blength*3,0,bheight*2+2])
	brick();
translate([blength*4,0,bheight*2+2])
echo([blength*4,0,bheight*2+2])
	brick();
// New layer.
translate([blength*0.5,0,bheight*3+3])
echo([blength*0.5,0,bheight*3+3])
	brick();
translate([blength*1.5,0,bheight*3+3])
echo([blength*1.5,0,bheight*3+3])
	brick();
translate([blength*2.5,0,bheight*3+3])
echo([blength*2.5,0,bheight*3+3])
	brick();
translate([blength*3.5,0,bheight*3+3])
echo([blength*3.5,0,bheight*3+3])
	brick();
translate([blength*4.5,0,bheight*3+3])
echo([blength*4.5,0,bheight*3+3])
	brick();
// New layer.
translate([0,0,bheight*4+4])
echo([0,0,bheight*4+4])
	brick();
translate([blength,0,bheight*4+4])
echo([blength,0,bheight*4+4])
	brick();
translate([blength*2,0,bheight*4+4])
echo([blength*2,0,bheight*4+4])
	brick();
translate([blength*3,0,bheight*4+4])
echo([blength*3,0,bheight*4+4])
	brick();
translate([blength*4,0,bheight*4+4])
echo([blength*4,0,bheight*4+4])
	brick();
