function bkgnd=calibrate(cam,x,y,width,height)
    disp("Calibrating background for image subtraction");
    img = snapshot(cam);
    bkgnd = img(x:x+height-1,y:y+width-1,:);
    for idx = 1:50
        img = snapshot(cam);
        img1 = insertShape(img,'Rectangle',[x y width height],'LineWidth',2,'Color','blue');
        imshow(img1);
        bkgnd = (bkgnd + img(x:x+height-1,y:y+width-1,:))/2;
    end
    disp("Calibrating Complete");