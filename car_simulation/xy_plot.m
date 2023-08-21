function xy_plot(data1, data3, data, data2)
persistent rawline estimation_line;

if isempty(rawline)
    rawline=animatedline('Color','r','Marker','+');
end
if isempty(estimation_line)
    estimation_line=animatedline('Color','b','Marker','*');
end

addpoints(rawline,data1,data3)
addpoints(estimation_line,data,data2)
end