function animation_wing_trajectory( x, n , R, c_bar, num, F)

global  point1_x_LE point1_z_LE point2_x_TE point2_z_TE point3_x_TE point3_z_TE
        point1_x_LE = zeros(n,1);
        point1_z_LE = zeros(n,1);
        point2_x_TE = zeros(n,1);
        point2_z_TE = zeros(n,1);
        point3_x_TE = zeros(n,1);
        point3_z_TE = zeros(n,1);

        scaler = 0.3*10^5;
        
        for j_LE = 1:1:n
            R01 = Rz(2*pi*x(j_LE,1))*Rx(2*pi*x(j_LE,2))*Ry(2*pi*x(j_LE,3));
            % point 1. Leading edge
            point1_x_LE(j_LE) = R01(1,:)*[0;R;0];
            point1_z_LE(j_LE) = R01(3,:)*[0;R;0];
                
            % point 2. Trailing edge
            point2_x_TE(j_LE) = R01(1,:)*[0;R;-c_bar];
            point2_z_TE(j_LE) = R01(3,:)*[0;R;-c_bar];
            
            point3_x_TE(j_LE) = R01(1,:)*[0;R;-c_bar*0.25];
            point3_z_TE(j_LE) = R01(3,:)*[0;R;-c_bar*0.25];            
            
                
            % plot(point1_x_LE,point1_z_LE,'.')
        end
       
        
            
       % color_plot_traj = colormap(autumn(n));    
       % 'Color',color_plot_traj(j,:)
       
       % k = 0;
       
       for j = n/40:n/40:(n/2);
            figure(1)
            axis('equal');
            hold on
            axis([-R*1.2 R*1.2 -R*1.2 R*1.2]);  
            axis off
            hold on
            
            %plot(point1_x_LE,point1_z_LE)
            plot([point1_x_LE(j) point2_x_TE(j)],[point1_z_LE(j) point2_z_TE(j)],'b','LineWidth',1.5)
            plot(point1_x_LE(j),point1_z_LE(j),'k.','LineWidth',8)
            
            plot_arrow(point3_x_TE(j),point3_z_TE(j),point3_x_TE(j)+F(j,1)*scaler,point3_z_TE(j)+F(j,3)*scaler,'LineWidth',1.5)
            
            plot([-60 60],[0 0],'k--')
            hold on
            % plot_arrow(0.75*R,0.5*R,-0.75*R,0.5*R,'LineWidth',1.5,'headwidth',0.025,'headheight',0.033)
            
            pause(0.01)
            %clf
       end
       
       for j = (n/2+n/40*(1)):n/40:(n);
            figure(2)
            axis('equal');
            hold on
            axis([-R*1.2 R*1.2 -R*1.2 R*1.2]);  
            axis off
            hold on
            
            %plot(point1_x_LE,point1_z_LE)
            plot([point1_x_LE(j) point2_x_TE(j)],[point1_z_LE(j) point2_z_TE(j)],'r','LineWidth',1.5)
            plot(point1_x_LE(j),point1_z_LE(j),'k.','LineWidth',8)
            plot([-60 60],[0 0],'k--')
            
            plot_arrow(point3_x_TE(j),point3_z_TE(j),point3_x_TE(j)+F(j,1)*scaler,point3_z_TE(j)+F(j,3)*scaler,'LineWidth',1.5)
            
            hold on
            % plot_arrow(-0.75*R,0.5*R,0.75*R,0.5*R,'LineWidth',1.5,'headwidth',0.025,'headheight',0.033)
            
            pause(0.01)
            %clf
       end
%        for j = (n-n/40*(k+1)):n/40:n;
%             figure(1)
%             axis('equal');
%             hold on
%             axis([-R*1.2 R*1.2 -R*0.75 R*0.75]);
%             hold on
%             
%             %plot(point1_x_LE,point1_z_LE)
%             plot([point1_x_LE(j) point2_x_TE(j)],[point1_z_LE(j) point2_z_TE(j)],'b')
%             plot(point1_x_LE(j),point1_z_LE(j),'k.','LineWidth',8)
%             plot([-R*1.5 R*1.5],[0 0],'k--')
%             
%             plot_arrow(point3_x_TE(j),point3_z_TE(j),point3_x_TE(j)+F(j,1)*scaler,point3_z_TE(j)+F(j,3)*scaler)
%             
%             pause(0.01)
%             %clf
%        end
       
            figure(3)
            axis('equal');
            hold on
            axis([-R*1.2 R*1.2 -R*1.2 R*1.2]);  
            axis off
            plot(point1_x_LE,point1_z_LE,'k','LineWidth',2)
            hold on
            plot([-R*1.5 R*1.5],[0 0],'k--','LineWidth',1.1)
       
       for j = n/40:n/40:(n/2);
            figure(3)
            axis('equal');
            hold on
            axis([-R*1.2 R*1.2 -R*1.2 R*1.2]);  
            axis off
            hold on
            
            %plot(point1_x_LE,point1_z_LE)
            plot([point1_x_LE(j) point2_x_TE(j)],[point1_z_LE(j) point2_z_TE(j)],'b','LineWidth',1.5)
                        
            pause(0.01)
            %clf
       end
       
       for j = (n/2+n/40*(1)):n/40:(n);
            figure(3)
            axis('equal');
            hold on
            axis([-R*1.2 R*1.2 -R*1.2 R*1.2]);  
            axis off
            hold on
            
            %plot(point1_x_LE,point1_z_LE)
            plot([point1_x_LE(j) point2_x_TE(j)],[point1_z_LE(j) point2_z_TE(j)],'r','LineWidth',1.5)
            
            pause(0.01)
            %clf
       end       
       

end

