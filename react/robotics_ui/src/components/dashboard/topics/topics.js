import { RenderType } from "./render_types"

export const Topics = ({component}) => {
    const ComponentToRender = RenderType[component.render_type] || RenderType.Generic;
    
    return (
        <div style={{ height: "90%" }}>
            {
                <ComponentToRender topic={component.topic} />
            }
		</div>
    );

};

    