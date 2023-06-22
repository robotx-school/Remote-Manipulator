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

let gripper = (pose) => {
    if (0 <= pose <= 255) {
        fetch("/api/gripper", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ pose: pose })
        }).then(function (res) {
            if (res.status !== 200) notify("Gripper error");
        })
    }
}

let gripper_minus = () => {
    let pose = parseInt(document.getElementById("gripper-percent").innerText);
    gripper(pose - 40);
}

let gripper_plus = () => {
    let pose = parseInt(document.getElementById("gripper-percent").innerText);
    gripper(pose + 40);
}

let robot_system_command = (cmd, extra="") => {
    fetch("/api/system", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({ command: cmd, extra: extra })
    }).then(function (res) {
        if (res.status !== 200) notify("Error while sending low level command", "error");
        else notify(`Command: ${cmd} executed!`, 'success');
    })
}


let shutdown = () => {
    Swal.fire({
        title: 'Do you want to shutdown robot ????',
        showDenyButton: false,
        showCancelButton: true,
        confirmButtonText: 'YES',
    }).then((result) => {
        if (result.isConfirmed) {
            Swal.fire('Shutdowned!', '', 'success');
            robot_system_command("shutdown");
        } else if (result.isDenied) {
            Swal.fire('Changes are not saved', '', 'info')
        }
    });
}

let robot_set_ip = () => {
    let new_ip = document.getElementById("robot-ip-input").value;
    fetch("/api/robot/set_ip", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({ ip: new_ip })
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
                document.getElementById("robot-mode-status").innerHTML = `<b>${data["mode"]}</b>`;
                document.getElementById("robot-ip-status").innerHTML = data["ip"];
                document.getElementById("robot-connected-status").innerHTML = data["connected"];
                document.getElementById("gripper-percent").innerHTML = data["gripper"];
            });
        }
    });
}, 1000);


document.addEventListener('keydown', function (event) {
    const key = event.key; // "a", "1", "Shift", etc.
    if (0) {
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
    }
});
