function SE2 = se2_to_SE2(se_2)
    SE2 = [cos(se_2(3)) -sin(se_2(3))  se_2(1);
           sin(se_2(3))  cos(se_2(3))  se_2(2);
           0               0                1 ];
end