init_mag=[-0.2    0.3   -100]';
init_mag=init_mag/norm(init_mag);
roll = atan2(init_mag(2),init_mag(3));    %roll
pitch = -asin(init_mag(1));           %pitch
Cbn = GetCbn([roll,pitch,0]);
Cbn*init_mag