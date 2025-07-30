
document.querySelectorAll(".graphs").forEach((graphslot, index) => {
    graphslot.addEventListener("mouseover", (event) => {
        //the online graph doesn't respond nicely to zooming in & out, so avoid it in that case

            graphslot.setAttribute("class", "graphs_zoomin")

    });
    
    graphslot.addEventListener("mouseout", (event) => {
        graphslot.setAttribute("class", "graphs")
    });
});
//just in case a dropdown is somehow already selected on website load
document.querySelectorAll(".graphs_zoomin").forEach((graphslot, index) => {
    graphslot.addEventListener("mouseover", (event) => {

            graphslot.setAttribute("class", "graphs_zoomin")

    });
    
    graphslot.addEventListener("mouseout", (event) => {
        graphslot.setAttribute("class", "graphs")
    });
});

