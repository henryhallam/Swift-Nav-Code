#include "simulator_data.h"
/* AUTO-GENERATED FROM simulator_almanac_generator.py */

u16 simulation_week_number = 1787;

double simulation_sats_pos[31][3];

double simulation_sats_vel[31][3];

u32 simulation_fake_carrier_bias[31];

u8 simulation_num_almanacs = 31;

const almanac_t simulation_almanacs[31] = {
{ 
  .gps = { 
    .ecc       = 0.002888,
    .toa       = 233472.000000,
    .inc       = 0.960802,
    .rora      = -0.000000,
    .a         = 26559936.201176,
    .raaw      = -0.818239,
    .argp      = 0.321128,
    .ma        = 1.872279,
    .af0       = 0.000008,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 1
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.013428,
    .toa       = 233472.000000,
    .inc       = 0.939314,
    .rora      = -0.000000,
    .a         = 26558889.374573,
    .raaw      = -0.846764,
    .argp      = -2.446993,
    .ma        = 2.488397,
    .af0       = 0.000488,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 2
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.016847,
    .toa       = 233472.000000,
    .inc       = 0.937600,
    .rora      = -0.000000,
    .a         = 26560867.285343,
    .raaw      = -2.021208,
    .argp      = 1.373169,
    .ma        = 2.664127,
    .af0       = 0.000349,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 3
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.010565,
    .toa       = 233472.000000,
    .inc       = 0.938349,
    .rora      = -0.000000,
    .a         = 26560248.233974,
    .raaw      = -0.830694,
    .argp      = 1.086847,
    .ma        = -0.403849,
    .af0       = 0.000007,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 4
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.003571,
    .toa       = 233472.000000,
    .inc       = 0.947841,
    .rora      = -0.000000,
    .a         = 26558667.938083,
    .raaw      = 0.220327,
    .argp      = 0.330886,
    .ma        = -1.481788,
    .af0       = -0.000380,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 5
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.007160,
    .toa       = 233472.000000,
    .inc       = 0.973727,
    .rora      = -0.000000,
    .a         = 26559553.699890,
    .raaw      = 2.342325,
    .argp      = -2.840829,
    .ma        = 1.241122,
    .af0       = 0.000333,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 7
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.013593,
    .toa       = 233472.000000,
    .inc       = 0.996982,
    .rora      = -0.000000,
    .a         = 26558003.623845,
    .raaw      = 2.440640,
    .argp      = -2.782502,
    .ma        = 0.687856,
    .af0       = 0.000012,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 8
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.017078,
    .toa       = 233472.000000,
    .inc       = 0.982259,
    .rora      = -0.000000,
    .a         = 26560046.921949,
    .raaw      = 2.321551,
    .argp      = 1.786317,
    .ma        = 2.326993,
    .af0       = 0.000319,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 9
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.013494,
    .toa       = 233472.000000,
    .inc       = 0.942640,
    .rora      = -0.000000,
    .a         = 26560233.133742,
    .raaw      = 0.237867,
    .argp      = 0.861145,
    .ma        = -1.187092,
    .af0       = -0.000121,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 10
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.014848,
    .toa       = 233472.000000,
    .inc       = 0.890131,
    .rora      = -0.000000,
    .a         = 26559946.261069,
    .raaw      = -1.138635,
    .argp      = 1.274041,
    .ma        = 1.431038,
    .af0       = -0.000479,
    .af1       = -0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 11
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.005056,
    .toa       = 233472.000000,
    .inc       = 0.987389,
    .rora      = -0.000000,
    .a         = 26559609.059849,
    .raaw      = -2.878973,
    .argp      = 0.384712,
    .ma        = 0.843477,
    .af0       = 0.000197,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 12
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.005360,
    .toa       = 233472.000000,
    .inc       = 0.977837,
    .rora      = -0.000000,
    .a         = 26559171.211665,
    .raaw      = 1.377231,
    .argp      = 2.220456,
    .ma        = -2.691997,
    .af0       = 0.000011,
    .af1       = -0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 13
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.007339,
    .toa       = 233472.000000,
    .inc       = 0.970431,
    .rora      = -0.000000,
    .a         = 26559508.410106,
    .raaw      = 1.345702,
    .argp      = -1.999234,
    .ma        = -2.357275,
    .af0       = 0.000183,
    .af1       = -0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 14
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.006239,
    .toa       = 233472.000000,
    .inc       = 0.937067,
    .rora      = -0.000000,
    .a         = 26559704.690166,
    .raaw      = 1.235251,
    .argp      = 0.273510,
    .ma        = -2.960076,
    .af0       = -0.000173,
    .af1       = -0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 15
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.007709,
    .toa       = 233472.000000,
    .inc       = 0.988000,
    .rora      = -0.000000,
    .a         = 26558783.686050,
    .raaw      = -2.860449,
    .argp      = 0.174079,
    .ma        = -1.328606,
    .af0       = -0.000224,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 16
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.009278,
    .toa       = 233472.000000,
    .inc       = 0.967908,
    .rora      = -0.000000,
    .a         = 26558793.756031,
    .raaw      = -1.827968,
    .argp      = -2.104891,
    .ma        = -2.298537,
    .af0       = -0.000083,
    .af1       = -0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 17
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.015035,
    .toa       = 233472.000000,
    .inc       = 0.926407,
    .rora      = -0.000000,
    .a         = 26559800.310347,
    .raaw      = 0.216093,
    .argp      = -2.049409,
    .ma        = -0.665323,
    .af0       = 0.000312,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 18
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.010157,
    .toa       = 233472.000000,
    .inc       = 0.965026,
    .rora      = -0.000000,
    .a         = 26559055.462854,
    .raaw      = -1.776980,
    .argp      = 0.411913,
    .ma        = -3.058934,
    .af0       = -0.000450,
    .af1       = -0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 19
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.005853,
    .toa       = 233472.000000,
    .inc       = 0.927390,
    .rora      = -0.000000,
    .a         = 26558320.675083,
    .raaw      = 0.162982,
    .argp      = 1.314663,
    .ma        = -0.106709,
    .af0       = 0.000177,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 20
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.021363,
    .toa       = 233472.000000,
    .inc       = 0.932255,
    .rora      = -0.000000,
    .a         = 26560842.114562,
    .raaw      = -0.829072,
    .argp      = -2.041057,
    .ma        = 0.327476,
    .af0       = -0.000354,
    .af1       = -0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 21
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.007021,
    .toa       = 233472.000000,
    .inc       = 0.924178,
    .rora      = -0.000000,
    .a         = 26560258.304234,
    .raaw      = 0.218072,
    .argp      = -2.064006,
    .ma        = -1.216956,
    .af0       = 0.000225,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 22
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.009099,
    .toa       = 233472.000000,
    .inc       = 0.952095,
    .rora      = -0.000000,
    .a         = 26558698.137662,
    .raaw      = 1.281210,
    .argp      = -2.798111,
    .ma        = 2.897357,
    .af0       = -0.000010,
    .af1       = -0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 23
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.002036,
    .toa       = 233472.000000,
    .inc       = 0.957081,
    .rora      = -0.000000,
    .a         = 26560555.238601,
    .raaw      = 2.308488,
    .argp      = 0.311778,
    .ma        = 2.115817,
    .af0       = -0.000027,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 24
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.003447,
    .toa       = 233472.000000,
    .inc       = 0.976124,
    .rora      = -0.000000,
    .a         = 26559397.690455,
    .raaw      = -2.920817,
    .argp      = 0.632882,
    .ma        = 0.064184,
    .af0       = 0.000021,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 25
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.021486,
    .toa       = 233472.000000,
    .inc       = 0.975243,
    .rora      = -0.000000,
    .a         = 26560006.661697,
    .raaw      = 1.368661,
    .argp      = 1.275835,
    .ma        = 2.792296,
    .af0       = 0.000114,
    .af1       = -0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 26
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.001075,
    .toa       = 233472.000000,
    .inc       = 0.962641,
    .rora      = -0.000000,
    .a         = 26559810.380522,
    .raaw      = -1.873221,
    .argp      = 0.052962,
    .ma        = -2.123342,
    .af0       = -0.000020,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 27
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.019157,
    .toa       = 233472.000000,
    .inc       = 0.986508,
    .rora      = -0.000000,
    .a         = 26559820.440391,
    .raaw      = -2.854617,
    .argp      = -1.716683,
    .ma        = -1.431703,
    .af0       = 0.000346,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 28
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.001677,
    .toa       = 233472.000000,
    .inc       = 0.968627,
    .rora      = -0.000000,
    .a         = 26560384.125949,
    .raaw      = -1.818947,
    .argp      = -0.929559,
    .ma        = 0.550541,
    .af0       = 0.000525,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 29
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.000489,
    .toa       = 233472.000000,
    .inc       = 0.959286,
    .rora      = -0.000000,
    .a         = 26560278.434452,
    .raaw      = 2.397483,
    .argp      = 2.656720,
    .ma        = 1.584534,
    .af0       = 0.000001,
    .af1       = -0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 30
  },
  .healthy   = 0,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.008084,
    .toa       = 233472.000000,
    .inc       = 0.978676,
    .rora      = -0.000000,
    .a         = 26561269.916374,
    .raaw      = 2.348260,
    .argp      = -0.630742,
    .ma        = 1.101170,
    .af0       = 0.000334,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 31
  },
  .healthy   = 1,
  .valid     = 1,
},
{ 
  .gps = { 
    .ecc       = 0.011426,
    .toa       = 233472.000000,
    .inc       = 0.948308,
    .rora      = -0.000000,
    .a         = 26560701.201705,
    .raaw      = 0.309108,
    .argp      = -0.173257,
    .ma        = 1.856852,
    .af0       = -0.000453,
    .af1       = 0.000000,
    .week      = 763
  },
  .sid = { 
    .constellation = 0,
    .band = 0,
    .sat = 32
  },
  .healthy   = 1,
  .valid     = 1,
},
};
