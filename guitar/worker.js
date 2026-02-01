import OpenSCAD from "./deps/openscad-2026.02.01/openscad.js";

async function load3mf(url) {
    const loader = new Promise(function (resolve, reject) {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", url);
        xhr.responseType = "blob";
        xhr.onload = function () {
        if (xhr.status >= 200 && xhr.status < 300) {
            const geometry = xhr.response;
            resolve(geometry);
        } else {
            reject({
            status: xhr.status,
            statusText: xhr.statusText
            });
        }
        };
        xhr.onerror = function () {
        reject({
            status: xhr.status,
            statusText: xhr.statusText
        });
        };
        xhr.send();
    });
    const blob = await loader;
    return blob.arrayBuffer();
}

function prepareCad([cad, body, pickguard, controlCover, neckPocket, neckPickup, bridgePickup, bridgeTool, bodyScript, pickguardScript, controlCoverScript]) {
    cad.FS.writeFile("/body.3mf", new Int8Array(body));
    cad.FS.writeFile("/pickguard.3mf", new Int8Array(pickguard));
    cad.FS.writeFile("/controlCover.3mf", new Int8Array(controlCover));

    cad.FS.writeFile("/neckPocket.3mf", new Int8Array(neckPocket));
    cad.FS.writeFile("/neckPickup.3mf", new Int8Array(neckPickup));
    cad.FS.writeFile("/bridgePickup.3mf", new Int8Array(bridgePickup));
    cad.FS.writeFile("/bridgeTool.3mf", new Int8Array(bridgeTool));

    cad.FS.writeFile("/bodyScript.scad", new Int8Array(bodyScript));
    cad.FS.writeFile("/pickguardScript.scad", new Int8Array(pickguardScript));
    cad.FS.writeFile("/controlCoverScript.scad", new Int8Array(controlCoverScript));
    return cad;
}

async function computeGuitar(msg) {
    const bodyReq = load3mf("body/batlike.3MF");
    const pickguardReq = load3mf("body/batlike-guard.3MF");
    const controlCoverReq = load3mf("body/batlike-cover.3MF");

    const neckPocketToolReq = load3mf("tool/neck/strat.3MF");
    const neckPickupToolReq = load3mf("tool/pickup/neck/humbucker.3MF");
    const bridgePickupToolReq = load3mf("tool/pickup/bridge/humbucker.3MF");
    const bridgeToolReq = load3mf("tool/bridge/musiclily-ultra.3MF");

    const bodyScriptReq = load3mf("scripts/body.scad");
    const pickguardScriptReq = load3mf("scripts/pickguard.scad");
    const controlCoverScriptReq = load3mf("scripts/control-cover.scad");

    const bodyCadReq = await OpenSCAD({noInitialRun: true});
    const pickguardCadReq = await OpenSCAD({noInitialRun: true});
    const controlCoverCadReq = await OpenSCAD({noInitialRun: true});

    const [bodyCad, pickguardCad, controlCoverCad, body, pickguard, controlCover, neckPocket, neckPickup, bridgePickup, bridgeTool, bodyScript, pickguardScript, controlCoverScript] = await Promise.all([
        bodyCadReq,
        pickguardCadReq,
        controlCoverCadReq,
        bodyReq,
        pickguardReq,
        controlCoverReq,
        neckPocketToolReq,
        neckPickupToolReq,
        bridgePickupToolReq,
        bridgeToolReq,
        bodyScriptReq,
        pickguardScriptReq,
        controlCoverScriptReq,
    ]);
    
    try {
        prepareCad([bodyCad, body, pickguard, controlCover, neckPocket, neckPickup, bridgePickup, bridgeTool, bodyScript, pickguardScript, controlCoverScript]);
        prepareCad([pickguardCad, body, pickguard, controlCover, neckPocket, neckPickup, bridgePickup, bridgeTool, bodyScript, pickguardScript, controlCoverScript]);
        prepareCad([controlCoverCad, body, pickguard, controlCover, neckPocket, neckPickup, bridgePickup, bridgeTool, bodyScript, pickguardScript, controlCoverScript]);
        
        bodyCad.callMain(["/bodyScript.scad", "-o", "/bodyOut.stl"]);
        const bodyModel = bodyCad.FS.readFile("/bodyOut.stl");
        
        pickguardCad.callMain(["/pickguardScript.scad", "-o", "pickguard-out.stl"]);
        const pickguardModel = pickguardCad.FS.readFile("/pickguard-out.stl");
        
        controlCoverCad.callMain(["/controlCoverScript.scad", "-o", "controlCover-out.stl"]);
        const controlCoverModel = controlCoverCad.FS.readFile("/controlCover-out.stl");

        const bodyModelBlob = new Blob([bodyModel], { type: "model/stl" });
        const pickguardModelBlob = new Blob([pickguardModel], { type: "model/stl" });
        const controlCoverModelBlob = new Blob([controlCoverModel], { type: "model/stl" });
        postMessage(
        '<a download="body.stl" href="' + URL.createObjectURL(bodyModelBlob) + '">Download the body model</a>, ' + 
        '<a download="pickguard.stl" href="' + URL.createObjectURL(pickguardModelBlob) + '">Download the pickguard model</a>, and ' +
        '<a download="controlCover.stl" href="' + URL.createObjectURL(controlCoverModelBlob) + '">Download the control cover model</a>' +
        '');
    }
    catch(err) {
        postMessage("failed");
        while (Error.isError(err)) {
            if (err.fileName !== undefined && err.fileName !== null && err.lineNumber !== undefined && err.lineNumber !== null) {
                console.log(`${err.name} - ${err.message} at ${err.fileName}:${err.lineNumber}`);
            } else {
                console.log(`${err.name} - ${err.message}`);
            }
            if (err.stack !== undefined && err.stack !== null) {
                console.log("Stack is:\n\n", err.stack);
            }
            err = err.cause;
        }
        if (err !== undefined && err !== null) {
            console.log(err);
        }
    }
}

addEventListener('message', computeGuitar);
