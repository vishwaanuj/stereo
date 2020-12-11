function generateQRCode() {

	var data = ""

	eel.generate_qr(data)(setImage)


function setImage(base64) {
	document.getElementById("real").src = base64[0];
	document.getElementById("depth").src = base64[1]
}

}
