for theta=0:pi/180:2*pi
	r=9.43*2.618/(theta+4.1);
	fprintf('%f %f 0.1\n',5+r*cos(pi/2-theta),5+r*sin(pi/2-theta));
end
