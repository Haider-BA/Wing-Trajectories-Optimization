% loop


num1 = 10;
num2 = 20;
num3 = 15;
num4 = 50;

total_num = num1*num2*num3*num4;
loop_data = zeros(total_num,5);
num = 1;

initial_torque = 0.8;
initial_kphi = 0;
initial_kpsi = 0;
initial_kth = 0;


for i_1 = 1:1:num1
    for i_2 = 1:1:num2
        for i_3 = 1:1:num3
            for i_4 = 3:1:num4   
                Obej_torque = initial_torque + 0.2*i_1;
                Obej_kphi = initial_kphi + 0.2*i_2;
                Obej_kpsi = initial_kpsi + 0.2*i_3;
                Obej_kth = initial_kth + 0.2*i_4;
                Obej = [Obej_torque Obej_kphi Obej_kpsi Obej_kth];
                ode_result = main(Obej);
                loop_data(num,:) = [ode_result Obej_torque Obej_kphi Obej_kpsi Obej_kth];
                num = num + 1;
            end
        end
    end
end

save loop.mat