#!/usr/bin/perl

use strict;
#this code is to verify a lookup + interpolate algorithm works adequately for
# angle averaging.

# plan will be to convert wind from r-theta (speed, angle)
# to xy   and then add the various xy terms to yield a reasonable average representation.

# E = Spd * sin(theta)
# N = Spd * cos(theta)

# can use library for final conversion back to angle and avg speed, as there will be one of them to
# quite a few sin and cos conversions.

# planning to use sin and cos values as 0..10k scaled integers.
# currently the system returns peak gust, total wind distance, and 'angle'
# every 8 minutes.
# lets return a speed weighted angle over that 8m,
# total distance as per now, peak gust, and average wind speed.

# I currently run the gust for 8.4s
# so there will be ~60 accumulations in the average.
# use a uint16_t for the gust count per 8.4s
# so need at least 24b (will use 32b) for the accumulation.

# the purpose here is to check my algorithm for sin, cos

my @sin_points;
my @sin_slope;
my $interp = 8;
my $interp_shift=5;
my $interp_and = 31;
# error is around 0.4% for 8 points. biggest % error is at 2, 0.81%, buts its an int rounding error.
&init_lookup($interp);

my $biggest_serr = 0;
my $biggest_serr_point=0;
my $biggest_cerr = 0;
my $biggest_cerr_point=0;

for (my $i = 0 ; $i <= 1023; $i++)
{
    my $angle = $i/512*3.14159;
    
    my $sinef=sin($angle)*10000;
    my $sine;
    if ($sinef >= 0) { $sine = int($sinef + 0.5);}
    else { $sine = int($sinef - 0.5);}
    my $sin = &sin($i);
    my $serr;
    
    if ($sine != 0) {
	$serr = 100*($sine - $sin) / $sine;
    } else {
	$serr=0;
    }
    
    if (abs($serr) > abs($biggest_serr))
    { $biggest_serr = $serr;
      $biggest_serr_point = $i;
    }

    my $cosf=cos($angle)*10000;
    my $cosine;
    if ($cosf >= 0) { $cosine = int($cosf + 0.5);}
    else { $cosine = int($cosf - 0.5);}
    my $cos = &cos($i);
    my $cerr;
    
    if ($cosine != 0) {
	$cerr = 100*($cosine - $cos) / $cosine;
    } else {
	$cerr=0;
    }
    
    if (abs($cerr) > abs($biggest_cerr))
    { $biggest_cerr = $cerr;
      $biggest_cerr_point = $i;
    }
    printf "i=%d sin=%d sinef=%1.2f sine=%d err=%1.2f%% cos=%d cosf=%1.2f cosine=%d err=%1.2f%%\n", $i, $sin, $sinef, $sine, $serr, $cos, $cosf, $cosine, $cerr;
}

printf "biggest serr = %1.2f at %d for %d interpolation points\n", $biggest_serr, $biggest_serr_point, $interp;  
printf "biggest cerr = %1.2f at %d for %d interpolation points\n", $biggest_cerr, $biggest_cerr_point, $interp;  


sub init_lookup
{
    my $interp = shift;
    my $step = 256 / $interp;
    print "run table init with interp = $interp points\n";
    # int, scaled by 10,000
    for (my $i=0; $i<=$interp; $i++) {
	$sin_points[$i] = int(10000*sin( ($i*$step)/512*3.14159265) + 0.5);
	printf "sin_points[%d] = %d\n", $i, $sin_points[$i];
    }
    for (my $i=0; $i<$interp; $i++) {
	$sin_slope[$i] = int(($sin_points[$i+1] - $sin_points[$i])/$step + 0.5);
	printf "sin_slope[%d] = %d\n", $i, $sin_slope[$i];
    }
}


sub sin {
    my $in = shift;
    if ($in > 512) {
	if ($in < 768)
	{   # 513 ..  .. 767
	    return 0 - sin_q1($in - 512);
	}
	else
	{
	    # 768 .. 1023
	    return 0 - sin_q1(1024 - $in);
	}
    }
    else
    {
	# <= 512
	if ($in <= 256)
	{   # 0 .. 256
	    return sin_q1($in);
	}
	else
	{
	    # 257 .. 512
	    return sin_q1(512 - $in);
	}
    }
}
    

sub cos {
    my $in = shift;
    if ($in > 512) {
	if ($in < 768)
	{   # 513 ..  .. 767
	    return 0 - sin_q1(768 - $in);
	}
	else
	{
	    # 768 .. 1023
	    return sin_q1($in - 768);
	}
    }
    else
    {
	# <= 512
	if ($in <= 256)
	{   # 0 .. 256
	    return sin_q1(256 - $in);
	}
	else
	{
	    # 257 .. 512
	    return 0 - sin_q1($in - 256);
	}
    }
}
    

    
sub sin_q1 {
    # returns sin for angles between 0 and 256, the adc representation of 90 degrees.

    # use 8 points lookup table and linear interpolation.
    
    my $in = shift;
    my $in_lookup = int($in >> $interp_shift);
    my $in_delta = int($in & $interp_and);

#    printf "%d %d %d\n", $in_lookup, $in_delta, $sin_points[$in_lookup];
    
    return ($sin_points[$in_lookup] + $sin_slope[$in_lookup] * $in_delta);
} 
    
