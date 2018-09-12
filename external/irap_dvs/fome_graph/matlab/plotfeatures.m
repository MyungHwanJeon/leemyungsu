function plotfeatures(featuresidx)
    [xpoints, ypoints] = uvfromidx(featuresidx,346);
    scatter(xpoints,ypoints,10,'k');
    drawnow
end