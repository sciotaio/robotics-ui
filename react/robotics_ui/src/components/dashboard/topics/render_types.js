import { TopicString } from "../../../plugins/String_topic/topic_String";
import { MapCenter } from "../../../plugins/GoogleMaps_Integration/Map_Center";
import { OpenStreetMapCenter } from "../../../plugins/OpenStreetMap_Integration/OpenStreetMap_Center";
import { GenericTopic } from "../../../plugins/Generic_topic/generic";

export const RenderType = Object.freeze({
	String: TopicString,
	MapCenter: MapCenter,
	OpenStreetMapCenter: OpenStreetMapCenter,
	Generic: GenericTopic,
});

export const RenderTypeTopicDefault = Object.freeze({
	String: {
		topic: [
			{
				name: "topic1",
				topic: "/topic1",
				type: "String",
				route: "/subscriber/topic1",
			},
		],
	},
    OpenStreetMapCenter: {
        topic: [
            {
                name: "gps_cordinates",
                topic: "/gps/fix",
                type: "NavSatFix",
                route: "/subscriber/gps_cordinates",
            },
            {
                name: "waypoint_reached",
                topic: "/waypoint_reached",
                type: "String",
                route: "/subscriber/waypoint_reached",
            },
            {
                name: "waypoint_add",
                topic: "/waypoint_add",
                type: "String",
                route: "/publisher/waypoint_add",
            },
            {
                name: "waypoint_delete",
                topic: "/waypoint_delete",
                type: "String",
                route: "/publisher/waypoint_delete",
            },
        ],
    },
    Generic: {
        topic: [
            {
                name: "topic1",
                topic: "/topic1",
                type: "String",
                route: "/subscriber/topic1",
            },
        ],
    },
});
