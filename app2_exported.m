classdef app2_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure              matlab.ui.Figure
        GridLayout            matlab.ui.container.GridLayout
        CheckBoxTarget        matlab.ui.control.CheckBox
        ObstacleButton        matlab.ui.control.Button
        CheckBoxObstacle      matlab.ui.control.CheckBox
        CheckBoxDrone         matlab.ui.control.CheckBox
        DroneButton           matlab.ui.control.Button
        StartButton           matlab.ui.control.Button
        label                 matlab.ui.control.Label
        TargetButton          matlab.ui.control.Button
        ObstaclesReadyButton  matlab.ui.control.Button
        UIAxes                matlab.ui.control.UIAxes
    end

    
    properties (Access = public)
        % map üzerindeki hedef drone ve engellerin seçilip seçilmediğini
        % kontrol eden 1 ve 0 değerlerini alan kontrol değişkenleri
        target_selected
        obstacle_selected
        drone_selected
        
        
        %her bir objenin konumunun tutulduğu değişkenler
        loc_target
        loc_obstacles
        loc_drone
        
        %harita olarak x ve y koordinatlaırnın ölçüsünde (şu an 10x10) bir
        %matris oluşturulmuş ve her bir koordinata ilgili objenin değeri
        %yazılması için oluşturulmuş değişken [10x10 matris] yani aslında
        %haritanın değişken anlamında bir modeli
        grid     % 0:empty -1:obstacle 1:target 2:drone
        
        % a star algoritmaasının tasarımı için kullanılan iki liste tipi
        % değişken
        CLOSED_LIST %kapalı listesi keşfedilen yolları saklar
        OPEN_LIST %open listesi keşfedilen yolları saklar ve ayrıca her bir
        %node un bir üst node 'unu da (parent) saklar
        
        %%%%%%%%%%%%%%%%%%%%%%%%%OPEN
        %%%%%%%%%%%%%%%%%%%%%%%%%LIST%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
        
        %%%%CLOSED LIST%%%%%%
        %X val | Y val |
        
        
        %count'lar bu listelere yeni bir node eklemek için ya da indis
        %bulmak için kullanılır yani listenin o anki satır sayısını tutar
        open_count
        closed_count
        
        
        xy_limit % haritanın x ve y limitlerini tutar
        f %toplam maliyet fonskiyonu
        g %bir sonraki node için maliyet fonksiyonu
        h %hedef e gitmek için gereken maliyet (sezgisel fonksiyon)
        path_cost
        
        %yapılan işlemlerde kullanılmak istenen nodu'un eşitlendiği o anki
        %node'un x ve y konumlarınının tutulduğu değişkenler
        current_node_x
        current_node_y
        
        
        target_drone_distance %hedef ve drone arasındaki mesafe
        path_available % hedef ve drone arasında mümkün bir yolup olup olmadığnı tutan değişken
        
        % komşu nodeların tutulduğu liste tipi değişken ve onun sayacı
        neighbors_array
        neighbors_count
        
        %minimumm maaliyeti bulmak için kullanılan indis
        ind_min_cost
        
        %en iyi (en kısa) yolu oluşturan nodeların konumlarının tutulduğu
        %değişken
        optimal_path
        
    end
    
    
    
    
    
    methods (Access = public)
    end
    
    methods (Access = private)
        %get_distance fonksiyonu verilen iki node'un arasındaki mesafeyi
        %hesaplayan fonksiyondur. Bu mesafe ayrıca h değerini verir. Bu
        %algoritmadı öklid bağıntısı kullanıldı
        
        function results = get_distance(app,x1,y1,x2,y2)
            results = sqrt(power(x1-x2,2) +power(y1-y2,2));
            
        end
        
        
        
        
        %Başlangıçta tek sefer yapılan bu işlem kapalı listesini oluşturmak
        %içindir. Başlangıçta kapalı listesinin ilk üyeleri engeller
        %(obstacles) olmalıdır çünkü bu nodelara hiç bir zaman
        %gidilemeyecektir. Grid matrisinde engelleri -1 ile belirlediğimiz
        %için grid matrisindeki -1 değerleri bölümlerin satır ve sutunun
        %yani x ve y konumlarını iç içe bir döngüyle kapalı listesine alıyoruz.
        
        function CLOSED_INITIAL(app)
            % to put the obstacle in CLOSED LIST
            k=1;
            for i=1:app.xy_limit
                for j=1:app.xy_limit
                    if(app.grid(i,j)==-1)
                        app.CLOSED_LIST(k,1)=i;
                        app.CLOSED_LIST(k,2)=j;
                        k=k+1;
                    end
                end
            end
            app.closed_count=size(app.CLOSED_LIST,1);
            
            
        end
        
        
        %Kapalı listesine engelleri ekledikten sonra bakılması gereken yani
        %açık listesine eklenmesi gereken ilk node drone'un bulunduğu
        %nodedur. OPEN LIST için gerekli bilgiler gerekli fonksyionlar
        %yardımıyla hesaplanır. h sezgisel fonksiyonu ilk adım olduğu için
        %sıfırdır. g maaliyet fonksiyonu hedef ve drone arasındaki mesafe
        %olduğu için get_distance fonksiyonuyla hesaplanır. F maaliyeti ise
        %bu ikisinin toplamadır. Böylelikle Açık listesinin ilk elemanı
        %drone node'u olmuş olur ve add_to_open fonksiyonuyla tüm bilgiler
        %girilerek eklenmiş olur.
        %Daha sonra bu ilk eleman incelendiği için open listdeki ilk sütun
        %(yani hala açık listesinde olup olmadığını belirten 1 ve 0
        %değerini alan değişkenin değeri) 0 yapılır ve bu node close liste
        %aktarılır.
        
        
        function  set_first_node(app)
            app.current_node_x = app.loc_drone(1);
            app.current_node_y=app.loc_drone(2);
            app.open_count=1;
            
            app.target_drone_distance=get_distance(app,app.current_node_x,...
                app.current_node_y,app.loc_target(1),app.loc_target(2));
            
            app.h =0;
            app.g = app.target_drone_distance;
            app.f = app.g+app.h;
            app.OPEN_LIST(app.open_count,:)=add_to_open(app,app.current_node_x,...
                app.current_node_y,app.loc_drone(1),app.loc_drone(2),...
                app.h,app.g,app.f);
            
            %set close first node(drone node)
            app.OPEN_LIST(app.open_count,1)=0;
            %and add to the close list
            app.closed_count=app.closed_count+1;
            app.CLOSED_LIST(app.closed_count,1)=app.current_node_x;
            app.CLOSED_LIST(app.closed_count,2)=app.current_node_y;
            app.path_available=1;
            
        end
        
        
        %bu fonksiyon açık listesine nodeları düzenli bir şekilde eklemek
        %için yazılmıştır
        function  new_open_row=add_to_open(~,node_x,node_y,from_x,from_y,h,g,f)
            new_open_row=[1,8]; %create row
            new_open_row(1,1)=1; % is open
            new_open_row(1,2)=node_x; %x value of node
            new_open_row(1,3)=node_y;   %y value of node
            new_open_row(1,4)=from_x;
            new_open_row(1,5)=from_y;
            new_open_row(1,6)=h;
            new_open_row(1,7)=g;
            new_open_row(1,8)=f;
            
            
        end
        
        %bu fonksiyon komşu nodeları bulmak için yazılmıştır. Bu işlem için
        %iç içe iki döngüyle beraber bir node'un sağına soluna üstüne ve
        %altına bakılır ve bu nodelar çeitli kontrollerden geçer.
        %komşu nodeları bulurken dikkat edilmesi gerekenler vardır.
        % 1. verilen grafik sınırları içerisinde kalınması
        % gerekir (xylimits) çünkü kenar nodelarda araştırma yaparken grafik dışına çıkılmamalıdır.
        %2. seçilen node obstacle(engel) olmamalıdır bu da kontrol
        %edilmelidir.
        % Bunlar kontrol edildikten sonra komşu nodeların kapalı listesinde
        % olup olmadığı kontrol edilir. Ve böylelikle komşu nodelar elde
        % edilmiş olur.
        %Bu kontrolleri sağlayan komşu nodelar neighbors_nodes listesine
        %eklenir ve bu nodeların h g f fonksiyonları da eklenir.
        function neighbors_nodes = neighbors(app,x,y,h,loc_target_x,...
                loc_target_y,CLOSED_LIST,XYLIM)
        neighbors_nodes=[];
        neighb_count=1;
        ind_close=size(app.CLOSED_LIST,1);
        
        for move_x=1:-1:-1
            for move_y=1:-1:-1
                if(move_x~=move_y || move_x ~=0 )
                    n_x=x+move_x;
                    n_y=y+move_y;
                    if((n_x>0 && n_x<XYLIM) && (n_y>0 && n_y<XYLIM))
                        node_okay=1;
                        
                        for ind=1:ind_close
                            if(n_x == CLOSED_LIST(ind,1) && n_y== CLOSED_LIST(ind,2))
                                node_okay=0;
                            end
                        end
                        
                        if node_okay
                            neighbors_nodes(neighb_count,1)=n_x;
                            neighbors_nodes(neighb_count,2)=n_y;
                            neighbors_nodes(neighb_count,3)=h+get_distance(app,x,y,n_x,n_y);
                            neighbors_nodes(neighb_count,4)=get_distance(app,loc_target_x,loc_target_y,n_x,n_y);
                            neighbors_nodes(neighb_count,5)=neighbors_nodes(neighb_count,3)+neighbors_nodes(neighb_count,4);
                            neighb_count=neighb_count+1;
                        end
                        
                    end
                    
                end
            end
        end
        
        end
        
        
        %% Bu fonksiyon açık listesindeki maaliyetin en düşük olduğu indisi bulmak için yazılmıştır.
        %% bu bir döngü içerisinde çalıştığı için hedefe ulaşılıp ulaşılmadığı kontrol edilir.Ulaşılana
        %kadar bu fonksiyon çalıştırılır.
        function index_min = min_cost(app,OPEN_LIST,open_count,target_x,target_y)
            temp=[];
            k=1;
            flag=0;
            goal_index=0;
            
            for j=1:open_count
                if (OPEN_LIST(j,1)==1)
                    temp(k,:)=[OPEN_LIST(j,:) j];
                    if (OPEN_LIST(j,2)==target_x && OPEN_LIST(j,3)==target_y)
                        flag=1;
                        goal_index=j;
                    end
                    k=k+1;
                end
            end
            if flag==1
                index_min=goal_index;
            end
            
            if size(temp ~=0)
                [min_val,temp_min_ind]=min(temp(:,8)); %smallest fn values index in temp array
                index_min=temp(temp_min_ind,9);         % %smallest fn values index in open list
            else
                index_min=-1; %no path available
            end
            
            
            
        end
        
        
        % yukarıada açıklanan fonksiyonlar yardımıyla bir while döngüsünyle
        % algoritma çalıştırılır.While döngüsünün devam etme koşulu o anki
        % incelenen node'un x ve y konumlarının hedef konumunlarıyla aynı
        % olmamasıdır ve geçerli bir yolun olup olmamasıdır.
        function algorithm(app)
            while ((app.current_node_x~=app.loc_target(1) || app.current_node_y~=app.loc_target(2)) &&  app.path_available==1)
                app.neighbors_array=neighbors(app,app.current_node_x,app.current_node_y...
                    ,app.h,app.loc_target(1),app.loc_target(2),app.CLOSED_LIST,app.xy_limit);
                app.neighbors_count=size(app.neighbors_array);% açık listesi komşu nodelarla sürekli güncellenir.
                
                
                for i=1:app.neighbors_count
                    flag=0;
                    
                    for j=1:app.open_count
                        if(app.neighbors_array(i,1)==app.OPEN_LIST(j,2) && app.neighbors_array(i,2)==app.OPEN_LIST(j,3)) %bu komsu node eğer açık listesinde varsa çalışır.
                            app.OPEN_LIST(j,8)=min(app.OPEN_LIST(j,8),app.neighbors_array(i,5)); %f fonksiyonu güncellenir
                            
                            if app.OPEN_LIST(j,8)==app.neighbors_array(i,5) %eğer komşu node'un f fonksiyonu açık listesindekine eşitse
                                app.OPEN_LIST(j,4)=app.current_node_x;      %diğer parametreler de güncellenir
                                app.OPEN_LIST(j,5)=app.current_node_y;
                                app.OPEN_LIST(j,6)=app.neighbors_array(i,3);
                                app.OPEN_LIST(j,7)=app.neighbors_array(i,4);
                                
                            end
                            flag=1; %komşu node açık listesinde varsa flag değeri 1 olur alttaki döngüye girmez
                        end
                    end
                    if flag ==0 % bu komşu nodeun açık listesindeki olmadığı durumdur ve bu node açık listesine direkt olarak eklenir.
                        app.open_count=app.open_count+1;
                        app.OPEN_LIST(app.open_count,:)=add_to_open(app,app.neighbors_array(i,1),app.neighbors_array(i,2)...
                            ,app.current_node_x,app.current_node_y,app.neighbors_array(i,3),app.neighbors_array(i,4),app.neighbors_array(i,5));
                    end
                end
                
                %en küçük f fonksiyon değerini bulmak için min_cost
                %çalıştırılır
                app.ind_min_cost = min_cost(app,app.OPEN_LIST,app.open_count,app.loc_target(1),app.loc_target(2));
                if (app.ind_min_cost~=-1)
                    app.current_node_x=app.OPEN_LIST(app.ind_min_cost,2); %min_cost fonksiyonuyla bulunan indis yardımıyla en düşük
                    app.current_node_y=app.OPEN_LIST(app.ind_min_cost,3); % fn değerine sahip satır açık listesinden alınır.
                    app.h=app.OPEN_LIST(app.ind_min_cost,6); %update h
                    
                    app.closed_count=app.closed_count+1; %sayaç arttırma işlemi
                    app.CLOSED_LIST(app.closed_count,1)=app.current_node_x; % ve bu node incelendiği için kapalı liste aktarılır
                    app.CLOSED_LIST(app.closed_count,2)=app.current_node_y;
                    app.OPEN_LIST(app.ind_min_cost,1)=0;
                else
                    app.path_available=0; %while döngüsünden çıkmak için
                end
                
                
                
                
                
                
            end
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
        %bu fonksiyon algoritma sonun oluşturalan açık ve kapalı listeleri
        %tarayar optimal yolu vermek için yazılmıştır.
        function find_optimal_path(app)
            cnt=size(app.CLOSED_LIST,1);%eğer bir yol varsa kapalı listesindeki son satır hedef olacağı için bu node seçilir
            app.optimal_path=[];
            app.current_node_x=app.CLOSED_LIST(cnt,1);
            app.current_node_y=app.CLOSED_LIST(cnt,2);
            cnt=1;
            app.optimal_path(cnt,1)=app.current_node_x;% ve hedef nodu optimal yola eklenir
            app.optimal_path(cnt,2)=app.current_node_y;
            cnt=cnt+1;
            
            if ((app.current_node_x==app.loc_target(1)) && (app.current_node_y==app.loc_target(2)))
                n_node =0; %if koşulu o anki konumun hedef konumuyla aynı olup olmadığını kontrol eder
                
                %open listin 4 ve 5 satırları bir önceki gelinen node'un
                %konumlarını verdiği için parent nodu bu şekilde elde
                %edilir.
                parent_x=app.OPEN_LIST(node_index(app,app.OPEN_LIST,app.current_node_x,app.current_node_y),4);
                parent_y=app.OPEN_LIST(node_index(app,app.OPEN_LIST,app.current_node_x,app.current_node_y),5);
                
                %bu döngüyle incelenen node drone node'una gelene kadar
                %parentler incelenerek minimum maliyetle olan yol optimal
                %path listesine eklenmiş olur.
                
                while ((parent_x~=app.loc_drone(1)) || (parent_y~=app.loc_drone(2)))
                    app.optimal_path(cnt,1)=parent_x;
                    app.optimal_path(cnt,2)=parent_y;
                    
                    n_node=node_index(app,app.OPEN_LIST,parent_x,parent_y);
                    parent_x=app.OPEN_LIST(n_node,4);
                    parent_y=app.OPEN_LIST(n_node,5);
                    cnt=cnt+1;
                end
                
                j=size(app.optimal_path,1);
                
                %Visualization
                %plt=plot(app.UIAxes,app.optimal_path(j,1)+.5,app.optimal_path(j,2)+.5);
                %j=j-1;
                for i=j:-1:1 %liste hedeften drone' a olduğu için tersten alınır.
                    
                    %küreyle modelleme işlemi
                    [X, Y, Z,]=sphere;
                    
                    r=0.2;
                    X2=X*r;
                    Y2=Y*r;
                    Z2=Z*r;
                    surf(app.UIAxes,X2+app.optimal_path(i,1)+.5,Y2+app.optimal_path(i,2)+.5,Z2+2)
                    pause(.25);
                    
                    drawnow;
                end
                plot(app.UIAxes,app.optimal_path(:,1)+.5,app.optimal_path(:,2)+.5);
                app.label.Text="Path is done . You can rotate the map  with the button on the top right";
            else
                app.label.Text="No path exist";
                
                
                
            end
            
        end
        
        % bu fonksyion verilen konumun açık listesindeki hangi sırada
        % olduğunu bulmaya yarar
        function n_index = node_index(app,OPEN_LIST,current_x,current_y)
            
            i=1;
            
            while (OPEN_LIST(i,2)~=current_x || OPEN_LIST(i,3) ~=current_y)
                i=i+1;
            end
            n_index=i;
            
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.target_selected=0;
            app.obstacle_selected=0;
            app.drone_selected=0;
            app.loc_target=[];
            app.loc_obstacles=[];
            app.loc_drone=[];
            app.CLOSED_LIST=[];
            app.grid=zeros(app.UIAxes.XLim(end),app.UIAxes.YLim(end));
            hold(app.UIAxes,"on");
            app.xy_limit=app.UIAxes.XLim(end);
        end

        % Button pushed function: TargetButton
        function TargetButtonPushed(app, event)
            
            %tüm objeleri seçme işlemi drawrectangle(roi) işlemiyle
            %yapılmıştır
            
            view(app.UIAxes,2); %2 boyuttan bakmak için
            if app.target_selected ==0
                app.label.Text = "lütfen hedef konumunu belirtin";
                roi_target=drawrectangle(app.UIAxes);
                
                if isvalid(roi_target) %roi den konum bilgisi alınır
                    roi_target.Position(1)=floor(roi_target.Position(1));
                    roi_target.Position(2)=floor(roi_target.Position(2));
                    roi_target.Position(3)=1;
                    roi_target.Position(4)=1;
                    
                    
                    app.grid(roi_target.Position(1),roi_target.Position(2))=1;%grid listesine hedef konumu 1 olarak eklenir.
                    app.loc_target=[roi_target.Position(1),roi_target.Position(2)];% bu konum loc_target değişkeninde saklanır.
                    
                    
                    % Visualization
                    rectangle(app.UIAxes,"Position",roi_target.Position,"FaceColor",[0 1 0])
                    roi_target.Visible=0;
                    app.CheckBoxTarget.Value = 1 ;
                    app.TargetButton.BackgroundColor = "green";
                    app.label.Text = "please click the buttons and specify the coordinates of obstacle and drone";
                    view(app.UIAxes,3);
                    app.target_selected=1;
                    app.loc_target=[roi_target.Position(1),roi_target.Position(2)]
                    app.TargetButton.Enable=0;
                end
            end
            
        end

        % Button pushed function: ObstacleButton
        function ObstacleButtonPushed(app, event)
            view(app.UIAxes,2);
            %obstacle (hedefler) birden fazla olacağı için bu bir while
            %döngüsüyle sağlanır. While döngüsü obstacle_selected değeri
            %bir olana kadar devam eder. obstacle selected değeri "Obstacle
            %Ready"  butonuna basılınca 1 olur.
            while ~app.obstacle_selected
                
                app.label.Text = "lütfen duvarların konumunu belirtin";
                roi_obstacle=drawrectangle(app.UIAxes);
                
                
                
                if isvalid(roi_obstacle)
                    if app.obstacle_selected
                        roi_obstacle.Visible=0;
                        break
                    end
                    
                    roi_obstacle.Visible=0;
                    roi_obstacle.Position(1)=floor(roi_obstacle.Position(1));
                    roi_obstacle.Position(2)=floor(roi_obstacle.Position(2));
                    roi_obstacle.Position(3)=1;
                    roi_obstacle.Position(4)=1;
                    
                    
                    app.grid(roi_obstacle.Position(1),roi_obstacle.Position(2))=-1;% obstacleların konumları grid de -1 ile modellenir.
                    
                    
                    
                    
                    
                    
                    %Visualization
                    rectangle(app.UIAxes,"Position",roi_obstacle.Position,"FaceColor",[1 0 0])
                    
                    
                    
                    %engeller küplerlerle modellenmiştir
                    xc=roi_obstacle.Position(1)+.5;
                    yc=roi_obstacle.Position(2)+.5;
                    zc=0.5;    % coordinated of the center
                    L=1;                 % cube size (length of an edge)
                    alpha=0.1;           % transparency (max=1=opaque)
                    
                    X = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
                    Y = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
                    Z = [0 0 3 0 0 0; 0 0 3 0 0 0; 3 3 3 0 3 3; 3 3 3 0 3 3];
                    
                    C='red';                  % unicolor
                    
                    X = L*(X-0.5) + xc;
                    Y = L*(Y-0.5) + yc;
                    Z = L*(Z-0.5) + zc;
                    
                    fill3(app.UIAxes,X,Y,Z,C,'FaceAlpha',alpha);    % draw cube
                    
                    
                    
                    
                end
                
                
            end
            
        end

        % Callback function: ObstaclesReadyButton, UIAxes
        function ObstaclesReadyButtonPushed(app, event)
            app.obstacle_selected=1; % while döngüsünün bitmesi için gereken flag
            view(app.UIAxes,3);
            app.CheckBoxObstacle.Value = 1 ;
            app.ObstacleButton.BackgroundColor = "red";
            app.label.Text = "please click the buttons and specify the coordinates of obstacle and drone";
            app.ObstaclesReadyButton.Visible=0;
            app.ObstacleButton.Enable=0;
            
            
            
            
        end

        % Button pushed function: DroneButton
        function DroneButtonPushed(app, event)
            %% hedef konumu seçilmesi gereken işlemler drone içinde tekrarlanmıştır
            if app.drone_selected ==0
                view(app.UIAxes,2);
                app.label.Text = "lütfen drone konumunu belirtin";
                roi_drone=drawrectangle(app.UIAxes);
                
                if isvalid(roi_drone)
                    roi_drone.Position(1)=floor(roi_drone.Position(1));
                    roi_drone.Position(2)=floor(roi_drone.Position(2));
                    roi_drone.Position(3)=1;
                    roi_drone.Position(4)=1;
                    rectangle(app.UIAxes,"Position",roi_drone.Position,"FaceColor",[0 0 1])
                    
                    app.grid(roi_drone.Position(1),roi_drone.Position(2))=2;
                    app.loc_drone=[roi_drone.Position(1),roi_drone.Position(2)];
                    
                    %Visualization
                    roi_drone.Visible=0;
                    app.CheckBoxDrone.Value = 1 ;
                    app.DroneButton.BackgroundColor = "blue";
                    app.label.Text = "please click the buttons and specify the coordinates of obstacle and drone";
                    
                    app.UIAxes.View = [-45 63.23828125];
                    app.drone_selected=1;
                    
                end
                if app.drone_selected==1
                    view(app.UIAxes,3);
                    app.DroneButton.Enable=0;
                    
                end
            end
        end

        % Button pushed function: StartButton
        function StartButtonPushed(app, event)
            %start butonunun çalışması için gereken kontroller yapılır ve
            %ilgili tüm fonksiyonlar çalışır.
            if app.target_selected==1 && app.drone_selected ==1 && app.obstacle_selected==1
                CLOSED_INITIAL(app);
                set_first_node(app);
                
                
                
                %algorithm
                algorithm(app);
                
                %find optimal path
                find_optimal_path(app);
                app.StartButton.Enable=0;
            else
                app.label.Text="lütfen tüm tanımlamaları yapın";
            end
            
            
            
            %             assignin('base', 'grid', app.grid);
            %             assignin('base', 'closeed', app.CLOSED_LIST);
            %             assignin('base', 'eopenlist', app.OPEN_LIST);
            %             assignin('base', 'optimalpath', app.optimal_path);
            %             assignin('base', 'expanded_array', app.neighbors_array);
            
            
            
            
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 769 567];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.WindowState = 'maximized';

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {'2x', '1x', '2x', '1x', '2x', '1x', 96, '1.16x'};
            app.GridLayout.RowHeight = {'1x', 40, 22, 22, 22};
            app.GridLayout.ColumnSpacing = 5.55555555555556;
            app.GridLayout.RowSpacing = 11;
            app.GridLayout.Padding = [5.55555555555556 11 5.55555555555556 11];

            % Create CheckBoxTarget
            app.CheckBoxTarget = uicheckbox(app.GridLayout);
            app.CheckBoxTarget.Text = '';
            app.CheckBoxTarget.Layout.Row = 4;
            app.CheckBoxTarget.Layout.Column = 2;

            % Create ObstacleButton
            app.ObstacleButton = uibutton(app.GridLayout, 'push');
            app.ObstacleButton.ButtonPushedFcn = createCallbackFcn(app, @ObstacleButtonPushed, true);
            app.ObstacleButton.Layout.Row = [3 4];
            app.ObstacleButton.Layout.Column = 3;
            app.ObstacleButton.Text = 'Obstacle';

            % Create CheckBoxObstacle
            app.CheckBoxObstacle = uicheckbox(app.GridLayout);
            app.CheckBoxObstacle.Enable = 'off';
            app.CheckBoxObstacle.Text = '';
            app.CheckBoxObstacle.Layout.Row = 4;
            app.CheckBoxObstacle.Layout.Column = 4;

            % Create CheckBoxDrone
            app.CheckBoxDrone = uicheckbox(app.GridLayout);
            app.CheckBoxDrone.Text = '';
            app.CheckBoxDrone.Layout.Row = 4;
            app.CheckBoxDrone.Layout.Column = 6;

            % Create DroneButton
            app.DroneButton = uibutton(app.GridLayout, 'push');
            app.DroneButton.ButtonPushedFcn = createCallbackFcn(app, @DroneButtonPushed, true);
            app.DroneButton.Layout.Row = [3 5];
            app.DroneButton.Layout.Column = 5;
            app.DroneButton.Text = 'Drone';

            % Create StartButton
            app.StartButton = uibutton(app.GridLayout, 'push');
            app.StartButton.ButtonPushedFcn = createCallbackFcn(app, @StartButtonPushed, true);
            app.StartButton.Layout.Row = [4 5];
            app.StartButton.Layout.Column = [7 8];
            app.StartButton.Text = 'Start';

            % Create label
            app.label = uilabel(app.GridLayout);
            app.label.Layout.Row = 2;
            app.label.Layout.Column = [2 7];
            app.label.Text = 'please click the buttons and specify the coordinates of target , obstacle and drone';

            % Create TargetButton
            app.TargetButton = uibutton(app.GridLayout, 'push');
            app.TargetButton.ButtonPushedFcn = createCallbackFcn(app, @TargetButtonPushed, true);
            app.TargetButton.Layout.Row = [3 5];
            app.TargetButton.Layout.Column = 1;
            app.TargetButton.Text = 'Target';

            % Create ObstaclesReadyButton
            app.ObstaclesReadyButton = uibutton(app.GridLayout, 'push');
            app.ObstaclesReadyButton.ButtonPushedFcn = createCallbackFcn(app, @ObstaclesReadyButtonPushed, true);
            app.ObstaclesReadyButton.Layout.Row = 5;
            app.ObstaclesReadyButton.Layout.Column = 3;
            app.ObstaclesReadyButton.Text = 'Obstacles Ready';

            % Create UIAxes
            app.UIAxes = uiaxes(app.GridLayout);
            title(app.UIAxes, {'Map'; ''})
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.View = [-45 63.23828125];
            app.UIAxes.XLim = [1 10];
            app.UIAxes.YLim = [1 10];
            app.UIAxes.ZLim = [0 4];
            app.UIAxes.XTick = [1 2 3 4 5 6 7 8 9 10];
            app.UIAxes.YTick = [1 2 3 4 5 6 7 8 9 10];
            app.UIAxes.ZTick = [0 1 2 3];
            app.UIAxes.ZTickLabel = {'0'; '1'; '2'; '3'};
            app.UIAxes.XGrid = 'on';
            app.UIAxes.YGrid = 'on';
            app.UIAxes.ZGrid = 'on';
            app.UIAxes.Layout.Row = 1;
            app.UIAxes.Layout.Column = [1 8];
            app.UIAxes.ButtonDownFcn = createCallbackFcn(app, @ObstaclesReadyButtonPushed, true);

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = app2_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end