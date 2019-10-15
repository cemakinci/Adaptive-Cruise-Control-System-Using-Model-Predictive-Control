b = [1];
a = [0.2 1 0 0];
G = tf(b,a,'InputDelay',0.05);
G_d = c2d(G,0.1);
[num1, den1] = tfdata(G_d, 'v');
[A,B,C,D] = tf2ss(num1,den1);



