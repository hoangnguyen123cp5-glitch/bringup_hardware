function [pos, status] = delta_fk(theta1, theta2, theta3)
    % DELTA_FK: Động học thuận (Updated & Robust)
    % Đầu ra: pos là vector cột [x; y; z]
    
    %% 1. THÔNG SỐ CƠ KHÍ
    L = 194.1649;   % Tay trên
    l = 374.0;      % Tay dưới
    r = 30.0;       % Bán kính tấm động
    H_tool = 51.58; % Độ dài dụng cụ
    
    % --- TỌA ĐỘ B MỚI (Base Joints) ---
    B1 = [-81.8751,   79.4695,  0];       
    B2 = [ 81.7897,   79.4678,  0];      
    B3 = [-113.4364, -12.0425,  0];   
    B = [B1; B2; B3];
    
    %% 2. TÍNH TOÁN VỊ TRÍ KHUỶU TAY (J)
    % Đổi sang radian
    th = [theta1, theta2, theta3] * pi/180;
    
    J = zeros(3, 3);
    for i = 1:3
        % Góc phương vị của tay đòn
        phi = atan2(B(i,2), B(i,1));
        
        % Khoảng cách từ tâm đến khớp motor
        R_measured = sqrt(B(i,1)^2 + B(i,2)^2);
        
        % Bán kính ảo (Chiếu điểm J về gần tâm một đoạn bằng r)
        % Để đưa về bài toán giao cầu 3 điểm cơ bản
        f = R_measured - r;
        
        % Tọa độ khuỷu tay (J) trong không gian
        J(i,1) = (f + L * cos(th(i))) * cos(phi);
        J(i,2) = (f + L * cos(th(i))) * sin(phi);
        J(i,3) = B(i,3) - L * sin(th(i));
    end
    
    %% 3. TÌM GIAO ĐIỂM (TRILATERATION)
    % Tìm giao điểm của 3 mặt cầu tâm J1, J2, J3 bán kính l
    [x, y, z, status] = trilateration_robust(J(1,:), J(2,:), J(3,:), l, l, l);
    
    if status == 1
        % Trừ đi chiều dài Tool để ra tọa độ đầu hút
        pos = [x; y; z - H_tool]; 
    else
        % Trả về NaN để không vẽ bậy lên đồ thị
        pos = [NaN; NaN; NaN]; 
    end
end

%% HÀM PHỤ TRỢ: GIẢI GIAO ĐIỂM 3 MẶT CẦU
function [x, y, z, status] = trilateration_robust(P1, P2, P3, r1, r2, r3)
    % P1, P2, P3: Tọa độ tâm 3 mặt cầu (Row vectors)
    % r1, r2, r3: Bán kính 3 mặt cầu
    
    % Vector nối P1->P2
    temp1 = P2 - P1;
    d = norm(temp1); 
    
    % Kiểm tra lỗi trùng nhau
    if d == 0, status = 0; x=0; y=0; z=0; return; end
    
    % Vector đơn vị trục X cục bộ
    e_x = temp1 / d;
    
    % Vector nối P1->P3
    temp2 = P3 - P1;
    
    % Chiếu P3 lên trục X cục bộ
    i = dot(e_x, temp2);
    
    % Vector trục Y cục bộ
    temp3 = temp2 - i * e_x;
    j = norm(temp3);
    
    % Kiểm tra lỗi thẳng hàng (Collinear)
    if j == 0, status = 0; x=0; y=0; z=0; return; end
    
    e_y = temp3 / j;
    
    % Vector trục Z cục bộ (Vuông góc với mặt phẳng P1-P2-P3)
    e_z = cross(e_x, e_y);
    
    % Giải hệ phương trình tìm tọa độ cục bộ (Local Coordinates)
    x_val = (r1^2 - r2^2 + d^2) / (2*d);
    y_val = (r1^2 - r3^2 + i^2 + j^2) / (2*j) - (i/j)*x_val;
    
    % Tính Z cục bộ
    z_sq = r1^2 - x_val^2 - y_val^2;
    
    % Thêm dung sai nhỏ (Tolerance) để tránh lỗi số học khi z ~ 0
    if z_sq < -1e-5 
        status = 0; x=0; y=0; z=0; return;
    end
    
    if z_sq < 0
        z_sq = 0; % Chấp nhận sai số cực nhỏ
    end
    
    status = 1;
    z_local = sqrt(z_sq);
    
    % Tính ra 2 nghiệm trong không gian 3D (World Coordinates)
    % Nghiệm 1: Z hướng theo e_z
    Res1 = P1 + x_val*e_x + y_val*e_y + z_local*e_z;
    % Nghiệm 2: Z ngược hướng e_z
    Res2 = P1 + x_val*e_x + y_val*e_y - z_local*e_z;
    
    % --- CHỐT NGHIỆM ---
    % Robot Delta luôn làm việc ở phía dưới (Z âm hơn)
    if Res1(3) < Res2(3)
        P_final = Res1;
    else
        P_final = Res2;
    end
    
    x = P_final(1); 
    y = P_final(2); 
    z = P_final(3);
end