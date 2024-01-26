% tests extra data needed for dnep problems
% test when not all ne_branch branch ids are bus ids

function mpc = case3_dnep
mpc.version = '2';
mpc.baseMVA = 100.0;
mpc.power_flex_price = 0.0001;

mpc.bus = [
	2	 3	 110.0	 40.0	 0.0	 0.0	 1	    1.10000	   -0.00000	 240.0	 1	    1.10000	    0.90000;
	3	 2	 110.0	 40.0	 0.0	 0.0	 1	    0.92617	    7.25883	 240.0	 1	    1.10000	    0.90000;
	4	 2	 95.0	 50.0	 0.0	 0.0	 1	    0.90000	  -17.26710	 240.0	 2	    1.10000	    0.90000;
];

mpc.gen = [
	2	 148.067	 54.697	 1000.0	 -1000.0	 1.1	 100.0	 1	 2000.0	 0.0;
	3	 170.006	 -8.791	 1000.0	 -1000.0	 0.92617	 100.0	 1	 2000.0	 0.0;
	4	 0.0	 -4.843	 1000.0	 -1000.0	 0.9	 100.0	 1	 0.0	 0.0;
];

%column_names%	bus	pg	qg	qmax	qmin	vg	mbase	gen_status	pmax	pmin	construction_cost
mpc.ne_gen = [
	3	 50	 40	 1000.0	 -1000.0	 1.1	 100.0	 1	 2000.0	 0.0  1.0;
];

mpc.gencost = [
	2	 0.0	 0.0	 3	   0.110000	   5.000000	   0.000000;
	2	 0.0	 0.0	 3	   0.085000	   1.200000	   0.000000;
	2	 0.0	 0.0	 3	   0.000000	   0.000000	   0.000000;
];

mpc.branch = [
	2	 3	 0.042	 0.9	 0.3	 9000.0	 0.0	 0.0	 0.0	 0.0	 1	 -30.0	 30.0;
];

%column_names%	f_bus	t_bus	br_r	br_x	br_b	rate_a	rate_b	rate_c	tap	shift	br_status	angmin	angmax	construction_cost
mpc.ne_branch = [
	2	 4	 0.065	 0.62	 0.45	 9000.0	 0.0	 0.0	 0.0	 0.0	 1	 -30.0	 30.0	 1;
	4	 3	 0.025	 0.75	 0.7	 50.0	 0.0	 0.0	 0.0	 0.0	 1	 -30.0	 30.0	 1;
	4	 3	 0.025	 0.75	 0.7	 0.0	 0.0	 0.0	 0.0	 0.0	 1	 -30.0	 30.0	 1;
];

% hours
mpc.time_elapsed = 1.0
mpc.tid = 24

%% storage data
%   storage_bus ps qs energy  energy_rating charge_rating  discharge_rating  charge_efficiency  discharge_efficiency  thermal_rating  qmin  qmax  r  x  p_loss  q_loss  status
mpc.storage = [
	 3	 0.0	 0.0	 20.0	 100.0	 50.0	 70.0	 0.8	 0.9	 100.0	 -50.0	 70.0	 0.1	 0.0	 0.0	 0.0	 1;
];

%% ne_storage data
%   storage_bus ps qs energy  energy_rating charge_rating  discharge_rating  charge_efficiency  discharge_efficiency  thermal_rating  qmin  qmax  r  x  p_loss  q_loss  status construction_cost
mpc.ne_storage = [
	 4	 0.0	 0.0	 20.0	 100.0	 50.0	 70.0	 0.8	 0.9	 100.0	 -50.0	 70.0	 0.1	 0.0	 0.0	 0.0	 1	10;
];
