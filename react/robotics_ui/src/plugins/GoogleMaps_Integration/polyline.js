/* global google */
import {
    forwardRef,
    useContext,
    useEffect,
    useImperativeHandle,
    useMemo,
    useRef
  } from 'react';
  
  import { GoogleMapsContext, useMapsLibrary } from '@vis.gl/react-google-maps';
  
  function usePolyline(props) {
    const {
      onClick,
      onDrag,
      onDragStart,
      onDragEnd,
      onMouseOver,
      onMouseOut,
      encodedPath,
      ...polylineOptions
    } = props;
  
    const callbacks = useRef({});
    Object.assign(callbacks.current, {
      onClick,
      onDrag,
      onDragStart,
      onDragEnd,
      onMouseOver,
      onMouseOut
    });
  
    const geometryLibrary = useMapsLibrary('geometry');
  
    const polyline = useRef(new google.maps.Polyline()).current;
  
    useMemo(() => {
      polyline.setOptions(polylineOptions);
    }, [polyline, polylineOptions]);
  
    const googleMapsContext = useContext(GoogleMapsContext);
    const map = googleMapsContext && googleMapsContext.map;
  
    useMemo(() => {
      if (!encodedPath || !geometryLibrary) return;
      const path = geometryLibrary.encoding.decodePath(encodedPath);
      polyline.setPath(path);
    }, [polyline, encodedPath, geometryLibrary]);
  
    useEffect(() => {
      if (!map) {
        if (map === undefined)
          console.error('<Polyline> has to be inside a Map component.');
  
        return;
      }
  
      polyline.setMap(map);
  
      return () => {
        polyline.setMap(null);
      };
    }, [map]);
  
    useEffect(() => {
      if (!polyline) return;
  
      const gme = google.maps.event;
      [
        ['click', 'onClick'],
        ['drag', 'onDrag'],
        ['dragstart', 'onDragStart'],
        ['dragend', 'onDragEnd'],
        ['mouseover', 'onMouseOver'],
        ['mouseout', 'onMouseOut']
      ].forEach(([eventName, eventCallback]) => {
        gme.addListener(polyline, eventName, (e) => {
          const callback = callbacks.current[eventCallback];
          if (callback) callback(e);
        });
      });
  
      return () => {
        gme.clearInstanceListeners(polyline);
      };
    }, [polyline]);
  
    return polyline;
  }
  
  export const Polyline = forwardRef((props, ref) => {
    const polyline = usePolyline(props);
  
    useImperativeHandle(ref, () => polyline, []);
  
    return null;
  });