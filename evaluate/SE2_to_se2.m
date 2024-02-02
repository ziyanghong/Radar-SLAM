function se2 = SE2_to_se2(SE2)
heading = atan2(SE2(2,1), SE2(1,1));
se2 = [SE2(1,3), SE2(2,3), heading];
end