
document.getElementById('FEAGI_launch').addEventListener('submit', start_feagi_wout_mon)
document.getElementById('feagi_dash').addEventListener('submit', set_burst_duration)

// document.getElementById('btn3').addEventListener('submit', set_burst_duration);
// document.getElementById('neuron_stats').addEventListener('change', set_stat_collections);
// document.getElementById('synapse_stats').addEventListener('change', set_stat_collections);
//

function start_feagi_wout_mon(e) {
    e.preventDefault();

    let connectome_path = document.getElementById('launch_path').value
    console.log(connectome_path)
    fetch('http://127.0.0.1:8000/v1/feagi/feagi/launch', {
     method: 'post',
     headers: {
         'Accept': 'application/json, text/plain, */*',
         'Content-type': 'application/json'
     },
        body:JSON.stringify({existing_connectome:connectome_path})
    })
        .then((res) => res.json())
        .then((data) => console.log(data)
        )

    let link1 = document.getElementById('feagi_launcher');
    link1.style.display = 'none';

    let link2= document.getElementById('feagi_dash');
    link2.style.display = 'block';
}



function set_burst_duration(e) {
    e.preventDefault();

    let burst_duration = document.getElementById('spike_interval').value
    console.log(burst_duration)
    fetch('http://127.0.0.1:8000/v1/feagi/feagi/burst_engine', {
     method: 'post',
     headers: {
         'Accept': 'application/json, text/plain, */*',
         'Content-type': 'application/json'
     },
        body:JSON.stringify({"burst_duration": burst_duration})
    })
        .then((res) => res.json())
        .then((data) => console.log(data)
        )
}

//
//
// function start_feagi_with_mon(e) {
//     e.preventDefault();
//
//     let connectome_path = document.getElementById('launch_path').value
//     console.log(connectome_path)
//     fetch('http://127.0.0.1:8000/v1/feagi/feagi/launch', {
//      method: 'post',
//      headers: {
//          'Accept': 'application/json, text/plain, */*',
//          'Content-type': 'application/json'
//      },
//         body:JSON.stringify({existing_connectome:connectome_path})
//     })
//         .then((res) => res.json())
//         .then((data) => console.log(data)
//         )
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
