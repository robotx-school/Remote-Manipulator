let notify = (text, type) => {
    Swal.fire({
        position: 'top-end',
        icon: type,
        title: text,
        showConfirmButton: false,
        timer: 1000
    });
}


let joystick_move = (dir) => {
    fetch("/api/joystick", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({ dir: dir })
    }).then(function (res) {
        if (res.status !== 200) notify("Error while sending direction", "error");
    });
}


let robot_system_command = (cmd) => {
    fetch("/api/system", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({ command: cmd })
    }).then(function (res) {
        if (res.status !== 200) notify("Error while sending low level command");
        else notify(`Command: ${cmd} executed!`);
    })
}

let upd_items = ["getl-table-x", "getl-table-y", "getl-table-z",
    "getl-table-r-0", "getl-table-r-1", "getl-table-r-2"];

document.getElementById("movel-btn").addEventListener('click', () => {
    let movel_input = document.getElementById("movel-input-data").value;
    fetch("/api/movel", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({ movel: movel_input })
    }).then(function (res) {
        if (res.ok)
            notify('MoveL sent', 'success');
        else if (res.status !== 200)
            notify('Error while sending movel', 'error');
    });
});



setInterval(function () {
    fetch("/api/get_data").then(function (res) {
        if (res.ok) {
            document.getElementById("getl-table-header").innerHTML = `Manipulator Coords (upd at ${new Date().toLocaleTimeString()})`;
            res.json().then(function (data) {
                document.getElementById("raw-coords").innerHTML = `RAW: ${data["getl"]}`;
                for (let i = 0; i < upd_items.length; i++) {
                    document.getElementById(upd_items[i]).innerText = data["getl"][i];
                }
                document.getElementById("robot-mode-status").innerHTML = data["mode"];
                document.getElementById("robot-ip-status").innerHTML = data["ip"];
            });
        }
    });
}, 2000);


document.addEventListener('keydown', function (event) {
    const key = event.key; // "a", "1", "Shift", etc.
    switch (key) {
        case 'ArrowUp':
            joystick_move("x+");
            break;
        case 'ArrowDown':
            joystick_move("x-");
            break;
        case 'ArrowLeft':
            joystick_move("y-");
            break;
        case 'ArrowRight':
            joystick_move("y+");
            break;

    }

});
