var storage = window.localStorage;   

function GotoVertical(editor)
{
	window.location.href = "verticalblocks.html?" + editor;
	storage.setItem("isHorizontal", "false");
}

