function [p_in1,p_in2] = getPath_init(p1,p2)
ag0 = -atan2(p2(2)-p1(2),p2(1)-p1(1));
ag1 = atan2(-50,150);
rot2 = @(x) [cos(x), -sin(x);sin(x),cos(x)];
ps_new = rot2(ag0+ag1)*[p1(1:2),p2(1:2)];
ps_in = rot2(-ag0-ag1)*[100,0;100,ps_new(2,2)-ps_new(2,1)]'+p1(1:2);
p_in1 = [ps_in(:,1);p1(3)] / 1000;
p_in2 = [ps_in(:,2);p1(3)] / 1000;
end
