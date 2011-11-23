diary on
fprintf('721\n');
for theta=0:pi/180:4*pi
	r=9.43*2.618/(theta+4.1);
	fprintf('%f %f 0.1\n',5+r*cos(pi/2-theta),-1+r*sin(pi/2-theta));
end
diary off
