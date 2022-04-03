
document.getElementById('FEAGI_launch').addEventListener('submit', start_feagi);
// document.getElementById('btn2').addEventListener('submit', snapshot_connectome);
// document.getElementById('btn3').addEventListener('submit', set_burst_duration);
// document.getElementById('neuron_stats').addEventListener('change', set_stat_collections);
// document.getElementById('synapse_stats').addEventListener('change', set_stat_collections);
//

function start_feagi(e) {
    e.preventDefault();

    let existing_connectome = document.getElementById('existing_connectome'.value)

    fetch('http://127.0.0.1:8000/v1/feagi/feagi/launch', {
     method: 'post',
     headers: {
         'Accept': 'application/json, text/plain, */*',
         'Content-type': 'application/json'
     },
        body:JSON.stringify({existing_connectome:existing_connectome})
    })
        .then((res) => res.json())
        .then((data) => console.log(data)
        )
}

//
// function start_feagi_old() {
//     let url = "http://127.0.0.1:8000/v1/feagi/feagi/launch";
//     let xhttp = new XMLHttpRequest();
//
//     xhttp.open("POST", url, true);
//
//     // Set the request header i.e. which type of content you are sending
//     xhttp.setRequestHeader("Content-Type", "application/json");
//
//     let data = JSON.stringify({"existing_connectome": document.getElementById('existing_connectome').value});
//     xhttp.send(data)
//
//     document.getElementById('FEAGI_settings').style.display = "compact";
// }
//
// function snapshot_connectome() {
//     let url = "http://127.0.0.1:8000/v1/feagi/connectome/snapshot";
//     let xhttp = new XMLHttpRequest();
//
//     xhttp.open("POST", url, true);
//
//     // Set the request header i.e. which type of content you are sending
//     xhttp.setRequestHeader("Content-Type", "application/json");
//
//     let data = JSON.stringify({"save_to_path": document.getElementById('save_to_path').value});
//     xhttp.send(data)
// }
//
// function set_burst_duration() {
//     let url = "http://127.0.0.1:8000/v1/feagi/feagi/burst_engine";
//     let xhttp = new XMLHttpRequest();
//
//     xhttp.open("POST", url, true);
//
//     // Set the request header i.e. which type of content you are sending
//     xhttp.setRequestHeader("Content-Type", "application/json");
//
//     let data = JSON.stringify({"burst_duration": document.getElementById('burst_duration').value});
//     xhttp.send(data)
// }
//
// function set_stat_collections() {
//     let url = "http://127.0.0.1:8000/v1/feagi/stats";
//     let xhttp = new XMLHttpRequest();
//
//     xhttp.open("POST", url, true);
//
//     // Set the request header i.e. which type of content you are sending
//     xhttp.setRequestHeader("Content-Type", "application/json");
//
//     let data = JSON.stringify({"neuron_stat_collection": true);
//     xhttp.send(data)
// }
