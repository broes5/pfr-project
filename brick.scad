include </home/benjamin/3D Printing/BOSL2/std.scad>

$fa=1;
$fs=0.4;

// Brick dimensions (cm).
bhight=7.6;
bwidth=11;
blength=23;
wallt=4;	// Wall thickness.

module brick() {
	difference() {
		cube([blength, bwidth, bhight], true);
		translate([0,0,-bhight/2])
			linear_extrude(bhight)
				rect([blength-wallt,bwidth-wallt], rounding=2);
	}
}

brick();
