

L = [80 ... go str
    50 ... clo
    50 ... cir
    50 ... go str
    60 ... clo
    130 ... cir
    20 ... go str
    70 ... clo
    40 ... cir
    20 ... go str
    ];
vx_ref = 5;
wp = wpt(L,vx_ref);

sim("track_generator.slx");

sim_time = tout;
x_ref = track.Data(:,1);
y_ref = track.Data(:,2);

figure;
plot(x_ref, y_ref);
grid on;


function wp_time = wpt(segment_length, vx)
time = 0;
    for i = 2:(length(segment_length)+1)
        time = time + segment_length(i-1)/vx;
        wp_time(i) = time;
    end

end
