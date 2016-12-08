function h = arc(x,y,r,nsegments)  
if nargin<4
    nsegments=50;
end
hold on
th = 0:pi/nsegments:pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit);
hold off
end

