import { useState, useEffect } from "react";

export const TopicString = ({ topic }) => {
	const [data, setData] = useState([]);

	useEffect(() => {
		const fetchData = () => {
			//console.log("Fetsching data");
			const route = topic[0].route.toString();
			//console.log("Route: ", route);
			fetch(route)
				.then((response) => response.json())
				.then((data) => {
					setData(data);
				});
		};

		fetchData(); // Fetch data immediately

		const intervalId = setInterval(fetchData, 5000); // Fetch data every 5 seconds

		// Clean up function
		return () => clearInterval(intervalId);
	}, [topic]);

	return (
		<div style={{ overflow: "hidden", textAlign: "left" }}>
			String Topic: {topic[0].name}
			<br />
			<pre>{JSON.stringify(data, null, 2) || "Loading..."}</pre>
		</div>
	);
};
