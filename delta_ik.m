function [theta1, theta2, theta3, status] = delta_ik(x, y, z)
    % DELTA_IK_NEW: Động học ngược (Đã cập nhật H_tool và B mới)
    
    %% 1. KHỞI TẠO
    theta1 = 0; theta2 = 0; theta3 = 0; status = 1;
    
    %% 2. THÔNG SỐ CƠ KHÍ
    L = 194.1649;   
    l = 374.0;      
    r = 30.0;       
    H_tool = 51.58; % (ĐÃ CẬP NHẬT)
    
    % --- TỌA ĐỘ B MỚI NHẤT ---
    B1 = [-81.8751,   79.4695,  0];       
    B2 = [ 81.7897,   79.4678,  0];      
    B3 = [-113.4364, -12.0425,  0];   
    B = [B1; B2; B3];
    % -------------------------
    
    z_plat = z + H_tool; % Tọa độ tâm tấm động
    theta_list = zeros(1, 3);
    
    for i = 1:3
        phi = atan2(B(i,2), B(i,1));
        R_measured = sqrt(B(i,1)^2 + B(i,2)^2);
        f = R_measured - r; 
        
        % Chuyển đổi tọa độ
        x_proj = x * cos(phi) + y * sin(phi);
        x_loc = x_proj - f; % Khoảng cách ngang từ trục motor đến khớp cầu
        
        y_loc = -x * sin(phi) + y * cos(phi);
        z_loc = z_plat - B(i,3);
        
        % Kiểm tra giới hạn tay dưới
        l_sq = l^2 - y_loc^2;
        if l_sq < 0, status = 0; return; end
        l_proj = sqrt(l_sq);
        
        % Giải phương trình: a*sin(th) + b*cos(th) = c
        a = 2 * z_loc * L;
        b = -2 * x_loc * L;
        c = l_proj^2 - L^2 - x_loc^2 - z_loc^2;
        
        delta_val = (c + b)^2 + (2*a)^2 - (c - b)*(c + b); % Rút gọn biểu thức delta
        % Hoặc dùng cách tính Delta truyền thống như file trước:
        A_q = c + b; B_q = -2*a; C_q = c - b;
        Delta = B_q^2 - 4*A_q*C_q;
        
        if Delta < 0
            status = 0; return;
        else
            t1 = (-B_q - sqrt(Delta)) / (2 * A_q);
            t2 = (-B_q + sqrt(Delta)) / (2 * A_q);
            
            th1 = 2 * atan(t1);
            th2 = 2 * atan(t2);
            
            % Chọn nghiệm Elbow Out (Khuỷu tay hướng ra ngoài)
            if (L * cos(th1)) > (L * cos(th2))
                theta_val = th1;
            else
                theta_val = th2;
            end
            theta_list(i) = rad2deg(theta_val);
        end
    end
    theta1 = theta_list(1); theta2 = theta_list(2); theta3 = theta_list(3);
end